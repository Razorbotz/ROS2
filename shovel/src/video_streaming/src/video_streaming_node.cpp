#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <linux/if_packet.h>
#include <thread>
#include <chrono>
#include <vector>
#include <list>
#include <linux/reboot.h>
#include <sys/reboot.h>
#include <cstdint>
#include <cerrno>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include "image_transport/image_transport.hpp"
#include <cv_bridge/cv_bridge.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/opt.h>
#include <libavutil/imgutils.h>
}

#include "messages/msg/talon_status.hpp"

// -------------------- Streaming / Encoder Globals --------------------
#define PORT 31338

// Global state
bool videoStreaming = false;
int  server_fd = -1;
rclcpp::Node::SharedPtr nodeHandle;
bool broadcast = true;

std::string robotName = "shovel";
std::string interfaceName = "wlP1p1s0";
int clientPort = 31338;

struct sockaddr_in client_addr;
socklen_t client_addr_len = sizeof(client_addr);
bool client_connected     = false;

// Simple stitched stream parameters
static const int STREAM_HEIGHT    = 400;
static const int STREAM_WIDTH     = 640;
static const int ZED_WIDTH        = 640;
static const int REALSENSE_WIDTH  = 640;
static const int STREAM_FPS       = 30;
static const int STREAM_BITRATE   = 2000000; // 4 Mbps

// FFmpeg H.264 encoder state
static AVCodecContext* h264_ctx   = nullptr;
static AVFrame*        video_frame = nullptr;
static AVPacket*       video_packet = nullptr;
static int64_t         frame_pts   = 0;
static std::mutex      encoder_mutex;

// Last images from each camera (grayscale)
std::mutex img_mutex;
cv::Mat last_zed_gray, last_rs_gray;

rclcpp::Time last_zed_stamp, last_rs_stamp;
bool showIntel = false;
static const int INTEL_THRESHOLD = 400;

std::atomic<uint64_t> last_video_tx_time_ms {0};
static std::atomic<bool> force_idr_next{false};

uint64_t get_time_ms() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

// -------------------- UDP Chunking Helper --------------------

bool send_udp_frame_chunked(int sock,
                            const uint8_t* data,
                            size_t len,
                            const struct sockaddr* dest_addr,
                            socklen_t addrlen,
                            uint16_t frame_id)
{
    const size_t CHUNK_SIZE = 1300; // fits in MTU
    uint16_t chunk_index = 0;
    uint16_t total_chunks = (len + CHUNK_SIZE - 1) / CHUNK_SIZE;

    size_t offset = 0;
    while (offset < len) {
        size_t bytes_to_send = std::min(len - offset, CHUNK_SIZE);

        struct FrameHeader {
            uint16_t frame_id;
            uint16_t chunk_index;
            uint16_t total_chunks;
        } header;

        header.frame_id     = htons(frame_id);
        header.chunk_index  = htons(chunk_index);
        header.total_chunks = htons(total_chunks);

        uint8_t packet[sizeof(header) + CHUNK_SIZE];
        memcpy(packet, &header, sizeof(header));
        memcpy(packet + sizeof(header), data + offset, bytes_to_send);

        last_video_tx_time_ms.store(get_time_ms(), std::memory_order_relaxed);
        sendto(sock, packet, sizeof(header) + bytes_to_send, 0, dest_addr, addrlen);

        offset += bytes_to_send;
        chunk_index++;
    }

    return true;
}

/**
 * @brief Initializes a low-latency H.264 (libx264) encoder.
 */
bool initialize_h264_encoder(int width, int height)
{
    if (h264_ctx) {
        return true;
    }

    const AVCodec* codec = avcodec_find_encoder_by_name("libx264");
    if (!codec) {
        codec = avcodec_find_encoder(AV_CODEC_ID_H264);
        if (!codec) {
            RCLCPP_ERROR(nodeHandle->get_logger(), "H.264 encoder not found (libx264/H264).");
            return false;
        }
    }

    h264_ctx = avcodec_alloc_context3(codec);
    if (!h264_ctx) {
        RCLCPP_ERROR(nodeHandle->get_logger(), "Could not allocate H.264 codec context.");
        return false;
    }

    // Encoder settings
    h264_ctx->width     = width;
    h264_ctx->height    = height;
    h264_ctx->pix_fmt   = AV_PIX_FMT_YUV420P;
    h264_ctx->time_base = { 1, STREAM_FPS };
    h264_ctx->framerate = { STREAM_FPS, 1 };
    h264_ctx->bit_rate  = STREAM_BITRATE;

    // No B-frames → no reordering → lower latency
    h264_ctx->max_b_frames = 0;

    // Low-latency x264 options
    av_opt_set(h264_ctx->priv_data, "preset", "ultrafast", 0);
    av_opt_set(h264_ctx->priv_data, "tune", "zerolatency", 0);
    av_opt_set(h264_ctx->priv_data, "profile", "baseline", 0);
    av_opt_set_int(h264_ctx->priv_data, "sync-lookahead", 0, 0);
    av_opt_set_int(h264_ctx->priv_data, "rc-lookahead", 0, 0);
    av_opt_set_int(h264_ctx->priv_data, "keyint", 5, 0);         // IDR every 5 frames
    av_opt_set_int(h264_ctx->priv_data, "force-cfr", 1, 0);
    av_opt_set_int(h264_ctx->priv_data, "crf", 26, 0);
    av_opt_set_int(h264_ctx->priv_data, "slice-max-size", 1200, 0);
    av_opt_set_int(h264_ctx->priv_data, "forced-idr", 1, 0);
    av_opt_set_int(h264_ctx->priv_data, "repeat-headers", 1, 0); // SPS/PPS before keyframes
    av_opt_set_int(h264_ctx->priv_data, "aud", 1, 0);

    if (avcodec_open2(h264_ctx, codec, nullptr) < 0) {
        RCLCPP_ERROR(nodeHandle->get_logger(), "Could not open H.264 codec.");
        avcodec_free_context(&h264_ctx);
        return false;
    }

    video_frame = av_frame_alloc();
    if (!video_frame) {
        RCLCPP_ERROR(nodeHandle->get_logger(), "Could not allocate video frame.");
        avcodec_free_context(&h264_ctx);
        h264_ctx = nullptr;
        return false;
    }

    video_frame->format = h264_ctx->pix_fmt;
    video_frame->width  = h264_ctx->width;
    video_frame->height = h264_ctx->height;

    if (av_frame_get_buffer(video_frame, 32) < 0) {
        RCLCPP_ERROR(nodeHandle->get_logger(), "Could not allocate frame buffer.");
        av_frame_free(&video_frame);
        avcodec_free_context(&h264_ctx);
        video_frame = nullptr;
        h264_ctx    = nullptr;
        return false;
    }

    // Pre-fill UV planes to neutral gray (128) once; we only change Y each frame.
    for (int y = 0; y < (video_frame->height / 2); ++y) {
        memset(video_frame->data[1] + y * video_frame->linesize[1], 128, video_frame->width / 2);
        memset(video_frame->data[2] + y * video_frame->linesize[2], 128, video_frame->width / 2);
    }

    video_packet = av_packet_alloc();
    if (!video_packet) {
        RCLCPP_ERROR(nodeHandle->get_logger(), "Could not allocate video packet.");
        av_frame_free(&video_frame);
        avcodec_free_context(&h264_ctx);
        video_frame = nullptr;
        h264_ctx    = nullptr;
        return false;
    }

    frame_pts = 0;
    RCLCPP_INFO(nodeHandle->get_logger(), "H.264 encoder initialized %dx%d @ %d FPS",
                width, height, STREAM_FPS);
    return true;
}

/**
 * @brief Cleans up H.264 encoder resources.
 */
void cleanup_h264_encoder(){
    if (h264_ctx) {
        avcodec_free_context(&h264_ctx);
        h264_ctx = nullptr;
    }
    if (video_frame) {
        av_frame_free(&video_frame);
        video_frame = nullptr;
    }
    if (video_packet) {
        av_packet_free(&video_packet);
        video_packet = nullptr;
    }
}

void send_zed_frame();

void zedImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg){
    try {
        cv::Mat img_bgr = cv_bridge::toCvCopy(msg, "bgr8")->image;
        if (img_bgr.empty()) return;

        cv::Mat gray_mat;
        cv::cvtColor(img_bgr, gray_mat, cv::COLOR_BGR2GRAY);

        {
            std::lock_guard<std::mutex> lk(img_mutex);
            last_zed_gray  = gray_mat.clone();
            last_zed_stamp = msg->header.stamp;
        }
        send_zed_frame();
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR(nodeHandle->get_logger(), "zedImageCallback exception: %s", e.what());
    }
}

void intelImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg){
    try {
        cv::Mat img_bgr = cv_bridge::toCvCopy(msg, "rgb8")->image;
        if (img_bgr.empty()) return;

        cv::Mat gray_mat;
        cv::cvtColor(img_bgr, gray_mat, cv::COLOR_RGB2GRAY);

        {
            std::lock_guard<std::mutex> lk(img_mutex);
            last_rs_gray  = gray_mat.clone();
            last_rs_stamp = msg->header.stamp;
        }
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR(nodeHandle->get_logger(), "intelImageCallback exception: %s", e.what());
    }
}


void send_zed_frame()
{
    if (!videoStreaming || server_fd < 0 || !client_connected) return;

    cv::Mat zed;
    {
        std::lock_guard<std::mutex> lk(img_mutex);
        if (last_zed_gray.empty()) return;
        zed = last_zed_gray;
    }

    {
        std::lock_guard<std::mutex> enc_lk(encoder_mutex);
        if (!h264_ctx) {
            if (!initialize_h264_encoder(STREAM_WIDTH, STREAM_HEIGHT)) {
                RCLCPP_ERROR(nodeHandle->get_logger(), "Failed to init H.264 encoder; stopping streaming.");
                videoStreaming = false;
                return;
            }
        }
    }

    cv::Mat zed_resized;
    // Resize zed to fill the entire stream window
    cv::resize(zed, zed_resized, cv::Size(STREAM_WIDTH, STREAM_HEIGHT), 0, 0, cv::INTER_AREA);

    {
        std::lock_guard<std::mutex> enc_lk(encoder_mutex);

        if (!h264_ctx || !video_frame || !video_packet) return;

        if (av_frame_make_writable(video_frame) < 0) {
            RCLCPP_WARN(nodeHandle->get_logger(), "Frame not writable.");
            return;
        }

        // Copy gray into Y plane row-by-row
        for (int y = 0; y < STREAM_HEIGHT; ++y) {
            memcpy(video_frame->data[0] + y * video_frame->linesize[0],
                   zed_resized.ptr(y),
                   STREAM_WIDTH);
        }

        video_frame->pts = frame_pts++;

        // Force an IDR immediately after a client (re)connect so decoding can start cleanly.
        if (force_idr_next.exchange(false, std::memory_order_relaxed)) {
            video_frame->pict_type = AV_PICTURE_TYPE_I;
            video_frame->key_frame = 1;
        } else {
            video_frame->pict_type = AV_PICTURE_TYPE_NONE;
            video_frame->key_frame = 0;
        }

        if (avcodec_send_frame(h264_ctx, video_frame) < 0) {
            RCLCPP_WARN(nodeHandle->get_logger(), "H.264 send_frame error.");
            return;
        }

        while (avcodec_receive_packet(h264_ctx, video_packet) == 0) {
            if (video_packet->size > 0) {
                uint16_t frame_id = static_cast<uint16_t>(frame_pts & 0xFFFF);
                send_udp_frame_chunked(server_fd,
                                       video_packet->data,
                                       static_cast<size_t>(video_packet->size),
                                       (struct sockaddr*)&client_addr,
                                       client_addr_len,
                                       frame_id);
            }
            av_packet_unref(video_packet);
        }
    }
}

void talon1Callback(const messages::msg::TalonStatus::SharedPtr talonStatus){
    if(talonStatus->sensor_position < INTEL_THRESHOLD){
        showIntel = true;
    }
    else{
        showIntel = false;
    }
}

std::string getAddressString(int family, std::string interfaceName){
    std::string addressString("");
    ifaddrs* interfaceAddresses = nullptr;
    for (int failed=getifaddrs(&interfaceAddresses); !failed && interfaceAddresses; interfaceAddresses=interfaceAddresses->ifa_next){
        if(strcmp(interfaceAddresses->ifa_name,interfaceName.c_str())==0 && interfaceAddresses->ifa_addr->sa_family == family) {
            if (interfaceAddresses->ifa_addr->sa_family == AF_INET) {
                sockaddr_in *socketAddress = reinterpret_cast<sockaddr_in *>(interfaceAddresses->ifa_addr);
                addressString += inet_ntoa(socketAddress->sin_addr);
            }
            if (interfaceAddresses->ifa_addr->sa_family == AF_INET6) {
                sockaddr_in6 *socketAddress = reinterpret_cast<sockaddr_in6 *>(interfaceAddresses->ifa_addr);
                for (int index = 0; index < 16; index += 2) {
                    char bits[5];
                    sprintf(bits,"%02x%02x", socketAddress->sin6_addr.s6_addr[index],socketAddress->sin6_addr.s6_addr[index + 1]);
                    if (index)addressString +=":";
                    addressString +=bits;
                }
            }
            if (interfaceAddresses->ifa_addr->sa_family == AF_PACKET) {
                sockaddr_ll *socketAddress = reinterpret_cast<sockaddr_ll *>(interfaceAddresses->ifa_addr);
                for (int index = 0; index < socketAddress->sll_halen; index++) {
                    char bits[3];
                    sprintf(bits,"%02x", socketAddress->sll_addr[index]);
                    if (index)addressString +=":";
                    addressString +=bits;
                }
            }
        }
    }
    freeifaddrs(interfaceAddresses);
    return addressString;
}



void broadcastIP() {
    int socketDescriptor = socket(AF_INET, SOCK_DGRAM, 0);
    if (socketDescriptor < 0) {
        RCLCPP_ERROR(nodeHandle->get_logger(), "Broadcast socket creation failed.");
        return;
    }

    while (rclcpp::ok()) {
        if (broadcast) {
            std::string addressString = getAddressString(AF_INET, interfaceName);
            if (addressString.empty()) {
                RCLCPP_WARN_THROTTLE(nodeHandle->get_logger(), *nodeHandle->get_clock(), 5000,
                    "Could not get IP for %s to broadcast.", interfaceName.c_str());
                std::this_thread::sleep_for(std::chrono::seconds(5));
                continue;
            }

            std::string message(robotName + "@" + addressString);
            
            struct sockaddr_in socketAddress;
            socketAddress.sin_family = AF_INET;
            socketAddress.sin_addr.s_addr = inet_addr("226.1.1.1");
            socketAddress.sin_port = htons(4322);

            struct in_addr localInterface;
            localInterface.s_addr = inet_addr(addressString.c_str());
            
            if (setsockopt(socketDescriptor, IPPROTO_IP, IP_MULTICAST_IF, (char *)&localInterface, sizeof(localInterface)) >= 0) {
                sendto(socketDescriptor, message.c_str(), message.length(), 0, (struct sockaddr *)&socketAddress, sizeof(socketAddress));
            }
        }
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }
    
    close(socketDescriptor);
}

static void sendVideoHeartbeat(int sock, const struct sockaddr* dest_addr, socklen_t addrlen) {
    const uint8_t hb[1] = {0};
    sendto(sock, hb, sizeof(hb), 0, dest_addr, addrlen);
    last_video_tx_time_ms.store(get_time_ms(), std::memory_order_relaxed);
}

int main(int argc, char **argv){
    rclcpp::init(argc,argv);

    nodeHandle = rclcpp::Node::make_shared("video_streaming");
    RCLCPP_INFO(nodeHandle->get_logger(),"Starting video streaming server node");

    // Parameters
    nodeHandle->declare_parameter<std::string>("interface_name", "wlP1p1s0");
    nodeHandle->declare_parameter<std::string>("robot_name", "shovel");
    nodeHandle->declare_parameter<int>("port", 31338);
    nodeHandle->get_parameter("interface_name", interfaceName);
    nodeHandle->get_parameter("robot_name", robotName);
    nodeHandle->get_parameter("port", clientPort);

    RCLCPP_INFO(nodeHandle->get_logger(), "interface_name: %s", interfaceName.c_str());
    RCLCPP_INFO(nodeHandle->get_logger(), "robot_name: %s", robotName.c_str());
    RCLCPP_INFO(nodeHandle->get_logger(), "port: %d", clientPort);

    image_transport::ImageTransport it(nodeHandle);
    auto zed_sub = nodeHandle->create_subscription<sensor_msgs::msg::Image>(
        "/zed_image",
        rclcpp::SensorDataQoS(),
        &zedImageCallback);

    auto intel_sub = nodeHandle->create_subscription<sensor_msgs::msg::Image>(
        "/d455i/color/image_raw",
        rclcpp::SensorDataQoS(),
        &intelImageCallback);

    auto talon1Subscriber = nodeHandle->create_subscription<messages::msg::TalonStatus>("talon_14_info",1,talon1Callback);

    ssize_t bytesRead;
    struct sockaddr_in address;
    int opt = 1;
    socklen_t addrlen = sizeof(address);
    uint8_t buffer[2048] = {0};

    if ((server_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        RCLCPP_FATAL(nodeHandle->get_logger(), "UDP Socket creation failed: %s", strerror(errno));
        return EXIT_FAILURE;
    }

    int broadcastEnable = 1;
    if (setsockopt(server_fd, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable)) < 0) {
        RCLCPP_WARN(nodeHandle->get_logger(), "Failed to enable broadcast: %s", strerror(errno));
    }

    // Reduce kernel buffering latency (but still allow some buffering)
    int sndbuf = 1 * 1024 * 1024;
    if (setsockopt(server_fd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf)) < 0) {
        RCLCPP_WARN(nodeHandle->get_logger(), "Failed to set send buffer: %s", strerror(errno));
    }

    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt))) {
        RCLCPP_ERROR(nodeHandle->get_logger(), "setsockopt SO_REUSEPORT failed: %s", strerror(errno));
        close(server_fd);
        return EXIT_FAILURE;
    }

    address.sin_family      = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port        = htons(clientPort);

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        RCLCPP_FATAL(nodeHandle->get_logger(), "Bind failed: %s", strerror(errno));
        close(server_fd);
        return EXIT_FAILURE;
    }
    RCLCPP_INFO(nodeHandle->get_logger(), "Server listening on port %d", clientPort);

    std::thread broadcastThread(broadcastIP);

    std::list<uint8_t> messageBytesList;
    uint8_t message[256];
    rclcpp::Rate rate(60);

    int flags = fcntl(server_fd, F_GETFL, 0);
    fcntl(server_fd, F_SETFL, flags | O_NONBLOCK);

    auto last_message_time = std::chrono::steady_clock::now();
    while(rclcpp::ok()){
        bytesRead = recvfrom(server_fd, buffer, sizeof(buffer), 0,
                             (struct sockaddr *)&client_addr, &client_addr_len);

        if (bytesRead > 0) {
            last_message_time = std::chrono::steady_clock::now();

            std::string received_str(reinterpret_cast<char*>(buffer), bytesRead);
            if (received_str == "Hello Robot") {
                std::string reply("Hello from server");
                sendto(server_fd, reply.c_str(), reply.length(), 0, (struct sockaddr *)&client_addr, client_addr_len);

                {
                    std::lock_guard<std::mutex> enc_lk(encoder_mutex);
                    cleanup_h264_encoder();
                    frame_pts = 0;
                    force_idr_next.store(true, std::memory_order_relaxed);
                }

                last_video_tx_time_ms.store(get_time_ms(), std::memory_order_relaxed);
                if (!client_connected) {
                    char client_ip[INET_ADDRSTRLEN];
                    inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, INET_ADDRSTRLEN);
                    RCLCPP_INFO(nodeHandle->get_logger(), "Received 'Hello Robot' from client at %s", client_ip);
                    client_connected = true;
                    broadcast = false;
                }
            }
            else {
                for(ssize_t i = 0; i < bytesRead; i++) {
                    messageBytesList.push_back(buffer[i]);
                }
            }
        }
        else if (bytesRead < 0 && (errno != EAGAIN && errno != EWOULDBLOCK)) {
            RCLCPP_ERROR(nodeHandle->get_logger(), "recvfrom failed: %s", strerror(errno));
        }

        auto now = std::chrono::steady_clock::now();
        if (client_connected && std::chrono::duration_cast<std::chrono::seconds>(now - last_message_time).count() > 5) {
           RCLCPP_WARN(nodeHandle->get_logger(), "Client timed out. Resuming broadcast.");
           client_connected = false;
           videoStreaming = false;
           broadcast = true;
        }

        if (client_connected) {
            uint64_t now_ms = get_time_ms();
            uint64_t last_tx = last_video_tx_time_ms.load(std::memory_order_relaxed);
            if (last_tx == 0) last_video_tx_time_ms.store(now_ms, std::memory_order_relaxed);
            if (now_ms - last_tx > 500) {
                sendVideoHeartbeat(server_fd, (struct sockaddr *)&client_addr, client_addr_len);
            }
        }

        while(messageBytesList.size()>0 && messageBytesList.front()<=messageBytesList.size()){
                int messageSize=messageBytesList.front();    
                messageBytesList.pop_front();
                messageSize--;
                for(int index=0;index<messageSize;index++){
                    message[index]=messageBytesList.front();
                    messageBytesList.pop_front();
                }
                uint8_t command=message[0];
                
                if(command==1){
                    videoStreaming=message[1];
                    RCLCPP_INFO(nodeHandle->get_logger(), "videoStreaming: %d", videoStreaming);
                    std::cout << "videoStreaming " << videoStreaming << std::endl;
                }
                else if(command==2){
                    uint8_t value = message[1];
                }
                else {
                }
            }

        rclcpp::spin_some(nodeHandle);
        rate.sleep();
    }

    RCLCPP_INFO(nodeHandle->get_logger(), "Shutting down video streaming server node.");
    cleanup_h264_encoder();
    if (server_fd >= 0) {
        close(server_fd);
    }
    broadcast = false;
    if (broadcastThread.joinable()) {
        broadcastThread.join();
    }
    rclcpp::shutdown();
    return 0;
}