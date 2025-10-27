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
#include <libswscale/swscale.h>
}

AVCodecContext* h265_encoder_ctx = nullptr;
AVFrame* video_frame = nullptr;
AVPacket* video_packet = nullptr;
SwsContext* sws_ctx = nullptr;
int64_t frame_pts = 0;

#define PORT 31338

bool videoStreaming = false;
int server_fd = -1;
rclcpp::Node::SharedPtr nodeHandle;
bool broadcast = true;
// cv::Mat img;
cv::Mat gray;
bool isGray = true;
int counter = 0;

struct sockaddr_in client_addr;
socklen_t client_addr_len = sizeof(client_addr);
bool client_connected = false;


/**
 * @brief Initializes the H.265 encoder. Call this when video streaming begins.
 * @param width The width of the video frames to be encoded.
 * @param height The height of the video frames to be encoded.
 * @return True on success, false on failure.
 */
bool initialize_h265_encoder(int width, int height) {
    const AVCodec* codec = avcodec_find_encoder_by_name("h265_nvenc");
    if (!codec) {
        // Fallback to default HEVC encoder if libx265 is not available
        codec = avcodec_find_encoder(AV_CODEC_ID_HEVC);
        if (!codec) {
            //RCLCPP_ERROR(nodeHandle->get_logger(), "H.265 encoder (libx265/HEVC) not found.");
            return false;
        }
    }

    h265_encoder_ctx = avcodec_alloc_context3(codec);
    if (!h265_encoder_ctx) {
        //RCLCPP_ERROR(nodeHandle->get_logger(), "Could not allocate video codec context.");
        return false;
    }

    // --- Set Encoder Parameters ---
    h265_encoder_ctx->width = width;
    h265_encoder_ctx->height = height;
    h265_encoder_ctx->pix_fmt = AV_PIX_FMT_YUV420P; // Standard for H.265
    h265_encoder_ctx->time_base = {1, 30}; // 30 FPS
    h265_encoder_ctx->framerate = {30, 1};

    // Set encoding options for low latency streaming
    av_opt_set(h265_encoder_ctx->priv_data, "preset", "ultrafast", 0);
    av_opt_set(h265_encoder_ctx->priv_data, "tune", "zerolatency", 0);

    if (avcodec_open2(h265_encoder_ctx, codec, nullptr) < 0) {
        //RCLCPP_ERROR(nodeHandle->get_logger(), "Could not open H.265 codec.");
        return false;
    }

    video_frame = av_frame_alloc();
    video_frame->format = h265_encoder_ctx->pix_fmt;
    video_frame->width = width;
    video_frame->height = height;
    if (av_frame_get_buffer(video_frame, 0) < 0) {
        //RCLCPP_ERROR(nodeHandle->get_logger(), "Could not allocate video frame data.");
        return false;
    }

    video_packet = av_packet_alloc();
    frame_pts = 0;

    //RCLCPP_INFO(nodeHandle->get_logger(), "H.265 encoder initialized successfully.");
    return true;
}

/**
 * @brief Cleans up and frees all H.265 encoder resources.
 */
void cleanup_h265_encoder() {
    if (h265_encoder_ctx) {
        avcodec_free_context(&h265_encoder_ctx);
        h265_encoder_ctx = nullptr;
    }
    if (video_frame) {
        av_frame_free(&video_frame);
        video_frame = nullptr;
    }
    if (video_packet) {
        av_packet_free(&video_packet);
        video_packet = nullptr;
    }
    if (sws_ctx) {
        sws_freeContext(sws_ctx);
        sws_ctx = nullptr;
    }
    //RCLCPP_INFO(nodeHandle->get_logger(), "H.265 encoder cleaned up.");
}


bool send_udp_frame_chunked(int sock, const uint8_t* data, size_t len, const struct sockaddr* dest_addr, socklen_t addrlen, uint16_t frame_id) {
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

        header.frame_id = htons(frame_id);
        header.chunk_index = htons(chunk_index);
        header.total_chunks = htons(total_chunks);

        uint8_t packet[sizeof(header) + CHUNK_SIZE];
        memcpy(packet, &header, sizeof(header));
        memcpy(packet + sizeof(header), data + offset, bytes_to_send);

        sendto(sock, packet, sizeof(header) + bytes_to_send, 0, dest_addr, addrlen);

        offset += bytes_to_send;
        chunk_index++;
    }

    return true;
}

std::mutex img_mutex;
cv::Mat last_zed_bgr, last_rs_bgr;
rclcpp::Time last_zed_stamp, last_rs_stamp;
const rclcpp::Duration SYNC_TOL = rclcpp::Duration::from_seconds(0.10);

void maybe_stitch_and_send();

void zedImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
    RCLCPP_INFO(nodeHandle->get_logger(), "zedCallback");
    try {
        cv::Mat img_bgr = cv_bridge::toCvCopy(msg, "bgr8")->image;
        if (img_bgr.empty()) return;

        {
            std::lock_guard<std::mutex> lk(img_mutex);
            last_zed_bgr = img_bgr.clone();
            last_zed_stamp = msg->header.stamp;
        }
        maybe_stitch_and_send();
    } catch (const std::exception &e) {
        RCLCPP_ERROR(nodeHandle->get_logger(), "zedImageCallback exception: %s", e.what());
    }
}

void intelImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
    RCLCPP_INFO(nodeHandle->get_logger(), "intelCallback");
    try {
        cv::Mat img_bgr = cv_bridge::toCvCopy(msg, "bgr8")->image;
        if (img_bgr.empty()) return;

        {
            std::lock_guard<std::mutex> lk(img_mutex);
            last_rs_bgr = img_bgr.clone();
            last_rs_stamp = msg->header.stamp;
        }
        maybe_stitch_and_send();
    } catch (const std::exception &e) {
        RCLCPP_ERROR(nodeHandle->get_logger(), "intelImageCallback exception: %s", e.what());
    }
}

void maybe_stitch_and_send()
{
    // Fast pre-check so we don't lock if we're clearly not streaming
    if (!videoStreaming || server_fd < 0) return;

    cv::Mat zed, rs;
    rclcpp::Time ts_zed, ts_rs;
    {
        std::lock_guard<std::mutex> lk(img_mutex);
        if (last_zed_bgr.empty() || last_rs_bgr.empty()) return;
        ts_zed = last_zed_stamp;
        ts_rs  = last_rs_stamp;
        if ((ts_zed - ts_rs).nanoseconds() > SYNC_TOL.nanoseconds() ||
            (ts_rs - ts_zed).nanoseconds() > SYNC_TOL.nanoseconds()) {
            // Not close enough in time yet
            return;
        }
        zed = last_zed_bgr.clone();
        rs  = last_rs_bgr.clone();
    }

    const int STREAM_HEIGHT = 400;
    const int ZED_WIDTH = 640, REALSENSE_WIDTH = 640;
    const int STITCHED_WIDTH = ZED_WIDTH + REALSENSE_WIDTH;

    // Lazy init encoder/colorspace when first needed
    if (!h265_encoder_ctx) {
        RCLCPP_INFO(nodeHandle->get_logger(), "Init H.265 %dx%d", STITCHED_WIDTH, STREAM_HEIGHT);
        if (!initialize_h265_encoder(STITCHED_WIDTH, STREAM_HEIGHT)) {
            RCLCPP_ERROR(nodeHandle->get_logger(), "H.265 init failed");
            videoStreaming = false;
            cleanup_h265_encoder();
            return;
        }
        sws_ctx = sws_getContext(STITCHED_WIDTH, STREAM_HEIGHT, AV_PIX_FMT_BGR24,
                                 STITCHED_WIDTH, STREAM_HEIGHT, h265_encoder_ctx->pix_fmt,
                                 SWS_BILINEAR, nullptr, nullptr, nullptr);
        if (!sws_ctx) {
            RCLCPP_ERROR(nodeHandle->get_logger(), "SWS init failed");
            videoStreaming = false;
            cleanup_h265_encoder();
            return;
        }
    }

    cv::Mat zed_resized, rs_resized, stitched;
    cv::resize(zed, zed_resized, cv::Size(ZED_WIDTH, STREAM_HEIGHT), 0, 0, cv::INTER_AREA);
    cv::resize(rs,  rs_resized,  cv::Size(REALSENSE_WIDTH, STREAM_HEIGHT), 0, 0, cv::INTER_AREA);
    cv::hconcat(zed_resized, rs_resized, stitched);

    const int stride[] = { static_cast<int>(stitched.step[0]) };
    sws_scale(sws_ctx, &stitched.data, stride, 0, stitched.rows,
              video_frame->data, video_frame->linesize);

    video_frame->pts = frame_pts++;

    if (avcodec_send_frame(h265_encoder_ctx, video_frame) < 0) {
        RCLCPP_WARN(nodeHandle->get_logger(), "H.265 send_frame error");
        return;
    }
    while (avcodec_receive_packet(h265_encoder_ctx, video_packet) == 0) {
        if (video_packet->size > 0) {
            uint16_t frame_id = frame_pts & 0xFFFF;
            send_udp_frame_chunked(server_fd, video_packet->data, video_packet->size,
                                   (struct sockaddr*)&client_addr, client_addr_len, frame_id);
        }
        av_packet_unref(video_packet);
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


std::string robotName="shovel";
void broadcastIP() {
    int socketDescriptor = socket(AF_INET, SOCK_DGRAM, 0);
    if (socketDescriptor < 0) {
        RCLCPP_ERROR(nodeHandle->get_logger(), "Broadcast socket creation failed.");
        return;
    }

    while (rclcpp::ok()) {
        if (broadcast) {
            std::string addressString = getAddressString(AF_INET, "wlP1p1s0");
            if (addressString.empty()) {
                RCLCPP_WARN_THROTTLE(nodeHandle->get_logger(), *nodeHandle->get_clock(), 5000, "Could not get IP for wlP1p1s0 to broadcast.");
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


int main(int argc, char **argv){
    rclcpp::init(argc,argv);

    nodeHandle = rclcpp::Node::make_shared("video_streaming");
    RCLCPP_INFO(nodeHandle->get_logger(),"Starting video streaming server node");

    image_transport::ImageTransport it(nodeHandle);
    auto zed_sub = nodeHandle->create_subscription<sensor_msgs::msg::Image>(
        "zed_image",
        rclcpp::SensorDataQoS(),
        &zedImageCallback);

    auto intel_sub = nodeHandle->create_subscription<sensor_msgs::msg::Image>(
        "/camera/camera/color/image_raw",
        rclcpp::SensorDataQoS(),
        &intelImageCallback);

    ssize_t bytesRead;
    struct sockaddr_in address;
    int opt = 1;
    socklen_t addrlen = sizeof(address);
    uint8_t buffer[2048] = {0};
    std::string hello("Hello from server");

    if ((server_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        RCLCPP_FATAL(nodeHandle->get_logger(), "UDP Socket creation failed: %s", strerror(errno));
        return EXIT_FAILURE;
    }

    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt))) {
        RCLCPP_ERROR(nodeHandle->get_logger(), "setsockopt failed: %s", strerror(errno));
        close(server_fd);
        return EXIT_FAILURE;
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        RCLCPP_FATAL(nodeHandle->get_logger(), "Bind failed: %s", strerror(errno));
        close(server_fd);
        return EXIT_FAILURE;
    }
    RCLCPP_INFO(nodeHandle->get_logger(), "Server listening on port %d", PORT);

    std::thread broadcastThread(broadcastIP);

    std::list<uint8_t> messageBytesList;
    uint8_t message[256];
    rclcpp::Rate rate(20);

    int flags = fcntl(server_fd, F_GETFL, 0);
    fcntl(server_fd, F_SETFL, flags | O_NONBLOCK);

    auto last_message_time = std::chrono::steady_clock::now();
    while(rclcpp::ok()){
        bytesRead = recvfrom(server_fd, buffer, sizeof(buffer), 0, (struct sockaddr *)&client_addr, &client_addr_len);

        if (bytesRead > 0) {
            last_message_time = std::chrono::steady_clock::now();
            
            std::string received_str(reinterpret_cast<char*>(buffer), bytesRead);
            if (received_str == "Hello Robot") {
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
                    std::cout << "videoStreaming " << videoStreaming << std::endl;
                }
                if(command==2){
                    uint8_t value = message[1];
                    isGray = (value % 2 == 0);
                }
            }

        rclcpp::spin_some(nodeHandle);
        rate.sleep();
    }

    RCLCPP_INFO(nodeHandle->get_logger(), "Shutting down video streaming server node.");
    if (server_fd >= 0) {
        close(server_fd);
    }
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