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
#include <condition_variable>
#include <atomic>
#include <algorithm>
#include <string>

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

// -------------------- Network / Globals --------------------
#define PORT 31338

bool videoStreaming = false;
int  server_fd = -1;
rclcpp::Node::SharedPtr nodeHandle;
bool broadcast = true;

std::string robotName = "shovel";
std::string interfaceName = "wlP1p1s0";
int clientPort = 31338;

struct sockaddr_in client_addr;
socklen_t client_addr_len = sizeof(client_addr);
bool client_connected = false;

std::condition_variable frame_cv;
bool new_frame_ready = false;
std::atomic<bool> keep_running{true};

std::mutex img_mutex;
cv::Mat last_zed_gray;
rclcpp::Time last_zed_stamp;

std::atomic<uint64_t> last_video_tx_time_ms{0};
std::atomic<bool> force_idr_next{false};
std::atomic<uint64_t> total_video_bytes_sent{0};

static std::mutex encoder_mutex;
static std::atomic<bool> reinit_encoder_requested{false};

// -------------------- Streaming Profiles --------------------
struct StreamProfile {
    int width;
    int height;
    int fps;
    int bitrate;
    int max_bitrate;
    int keyframe_interval;
    size_t chunk_size;
    bool clarity_mode;
    std::string name;
};

static StreamProfile clarity_profile{
    640,
    400,
    25,
    4000000,
    6000000,
    25,
    1200,
    true,
    "clarity"
};

static StreamProfile saver_profile{
    512,
    320,
    15,
    1800000,
    2200000,
    15,
    1100,
    false,
    "bandwidth_saver"
};

static std::mutex profile_mutex;
static StreamProfile active_profile = saver_profile;

// -------------------- Encoder State --------------------
static AVCodecContext* h264_ctx = nullptr;
static AVFrame* video_frame = nullptr;
static AVPacket* video_packet = nullptr;
static SwsContext* sws_ctx = nullptr;
static int64_t frame_pts = 0;

uint64_t get_time_ms() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

StreamProfile get_active_profile() {
    std::lock_guard<std::mutex> lk(profile_mutex);
    return active_profile;
}

void cleanup_h264_encoder() {
    if (sws_ctx) {
        sws_freeContext(sws_ctx);
        sws_ctx = nullptr;
    }
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

void apply_stream_profile(const StreamProfile& profile) {
    {
        std::lock_guard<std::mutex> lk(profile_mutex);
        active_profile = profile;
    }

    {
        std::lock_guard<std::mutex> enc_lk(encoder_mutex);
        cleanup_h264_encoder();
        frame_pts = 0;
        force_idr_next.store(true, std::memory_order_relaxed);
        reinit_encoder_requested.store(false, std::memory_order_relaxed);
    }

    RCLCPP_INFO(
        nodeHandle->get_logger(),
        "Applied stream profile: %s (%dx%d @ %d FPS, bitrate=%d, max=%d)",
        profile.name.c_str(),
        profile.width,
        profile.height,
        profile.fps,
        profile.bitrate,
        profile.max_bitrate
    );
}

bool initialize_h264_encoder(const StreamProfile& profile) {
    if (h264_ctx) return true;

    const AVCodec* codec = avcodec_find_encoder_by_name("libx264");
    if (!codec) {
        RCLCPP_ERROR(nodeHandle->get_logger(), "H.264 encoder not found.");
        return false;
    }

    h264_ctx = avcodec_alloc_context3(codec);
    if (!h264_ctx) {
        RCLCPP_ERROR(nodeHandle->get_logger(), "Failed to allocate H.264 context.");
        return false;
    }

    h264_ctx->width = profile.width;
    h264_ctx->height = profile.height;
    h264_ctx->time_base = {1, profile.fps};
    h264_ctx->framerate = {profile.fps, 1};
    h264_ctx->pix_fmt = AV_PIX_FMT_YUV420P;

    h264_ctx->bit_rate = profile.bitrate;
    h264_ctx->rc_max_rate = profile.max_bitrate;
    h264_ctx->rc_min_rate = std::max(profile.bitrate / 2, 300000);
    h264_ctx->rc_buffer_size = profile.bitrate;

    h264_ctx->gop_size = profile.keyframe_interval;
    h264_ctx->max_b_frames = 0;
    h264_ctx->thread_type = FF_THREAD_SLICE;

    av_opt_set(h264_ctx->priv_data, "preset", profile.clarity_mode ? "veryfast" : "ultrafast", 0);
    av_opt_set(h264_ctx->priv_data, "tune", "zerolatency", 0);
    av_opt_set_int(h264_ctx->priv_data, "rc-lookahead", 0, 0);
    av_opt_set_int(h264_ctx->priv_data, "sync-lookahead", 0, 0);
    av_opt_set_int(h264_ctx->priv_data, "intra-refresh", 1, 0);
    av_opt_set_int(h264_ctx->priv_data, "aud", 1, 0);

    std::string x264_params =
        "keyint=" + std::to_string(profile.keyframe_interval) +
        ":min-keyint=" + std::to_string(profile.keyframe_interval) +
        ":scenecut=0:repeat-headers=1:vbv-maxrate=" + std::to_string(profile.max_bitrate / 1000) +
        ":vbv-bufsize=" + std::to_string(profile.bitrate / 1000);

    av_opt_set(h264_ctx->priv_data, "x264-params", x264_params.c_str(), 0);
    av_opt_set_int(h264_ctx->priv_data, "slice-max-size", static_cast<int>(std::max<size_t>(400, profile.chunk_size - 100)), 0);

    if (avcodec_open2(h264_ctx, codec, nullptr) < 0) {
        RCLCPP_ERROR(nodeHandle->get_logger(), "Could not open H.264 codec.");
        avcodec_free_context(&h264_ctx);
        return false;
    }

    video_frame = av_frame_alloc();
    if (!video_frame) {
        RCLCPP_ERROR(nodeHandle->get_logger(), "Failed to allocate AVFrame.");
        avcodec_free_context(&h264_ctx);
        return false;
    }

    video_frame->format = h264_ctx->pix_fmt;
    video_frame->width = h264_ctx->width;
    video_frame->height = h264_ctx->height;

    if (av_frame_get_buffer(video_frame, 32) < 0) {
        RCLCPP_ERROR(nodeHandle->get_logger(), "Failed to allocate AVFrame buffer.");
        av_frame_free(&video_frame);
        avcodec_free_context(&h264_ctx);
        return false;
    }

    video_packet = av_packet_alloc();
    if (!video_packet) {
        RCLCPP_ERROR(nodeHandle->get_logger(), "Failed to allocate AVPacket.");
        av_frame_free(&video_frame);
        avcodec_free_context(&h264_ctx);
        return false;
    }

    sws_ctx = sws_getContext(
        profile.width, profile.height, AV_PIX_FMT_GRAY8,
        profile.width, profile.height, AV_PIX_FMT_YUV420P,
        SWS_BILINEAR, nullptr, nullptr, nullptr);

    if (!sws_ctx) {
        RCLCPP_ERROR(nodeHandle->get_logger(), "Failed to create sws context.");
        av_packet_free(&video_packet);
        av_frame_free(&video_frame);
        avcodec_free_context(&h264_ctx);
        return false;
    }

    frame_pts = 0;
    return true;
}

bool send_udp_frame_chunked(int sock,
                            const uint8_t* data,
                            size_t len,
                            const struct sockaddr* dest_addr,
                            socklen_t addrlen,
                            uint16_t frame_id)
{
    const StreamProfile profile = get_active_profile();
    const size_t CHUNK_SIZE = profile.chunk_size;

    uint16_t chunk_index = 0;
    uint16_t total_chunks = static_cast<uint16_t>((len + CHUNK_SIZE - 1) / CHUNK_SIZE);

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

        std::vector<uint8_t> packet(sizeof(header) + bytes_to_send);
        memcpy(packet.data(), &header, sizeof(header));
        memcpy(packet.data() + sizeof(header), data + offset, bytes_to_send);

        ssize_t sent = sendto(sock,
                              packet.data(),
                              packet.size(),
                              0,
                              dest_addr,
                              addrlen);

        if (sent < 0) {
            RCLCPP_WARN(nodeHandle->get_logger(), "sendto failed: %s", strerror(errno));
            return false;
        }

        total_video_bytes_sent.fetch_add(static_cast<uint64_t>(sent), std::memory_order_relaxed);
        last_video_tx_time_ms.store(get_time_ms(), std::memory_order_relaxed);

        offset += bytes_to_send;
        chunk_index++;

        if ((chunk_index % 4) == 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(profile.clarity_mode ? 150 : 300));
        }
    }

    return true;
}

void encode_and_send(const cv::Mat& zed) {
    if (!videoStreaming || server_fd < 0 || !client_connected || zed.empty()) return;

    const StreamProfile profile = get_active_profile();

    {
        std::lock_guard<std::mutex> enc_lk(encoder_mutex);
        if (!h264_ctx) {
            if (!initialize_h264_encoder(profile)) {
                RCLCPP_ERROR(nodeHandle->get_logger(), "Failed to init H.264 encoder; stopping streaming.");
                videoStreaming = false;
                return;
            }
        }
    }

    cv::Mat zed_resized;
    cv::resize(zed,
               zed_resized,
               cv::Size(profile.width, profile.height),
               0,
               0,
               profile.clarity_mode ? cv::INTER_LINEAR : cv::INTER_AREA);

    {
        std::lock_guard<std::mutex> enc_lk(encoder_mutex);

        if (!h264_ctx || !video_frame || !video_packet || !sws_ctx) return;
        if (zed_resized.type() != CV_8UC1) {
            RCLCPP_WARN(nodeHandle->get_logger(), "Expected mono8 frame.");
            return;
        }

        if (av_frame_make_writable(video_frame) < 0) {
            RCLCPP_WARN(nodeHandle->get_logger(), "Frame not writable.");
            return;
        }

        const uint8_t* src_slices[1] = { zed_resized.data };
        int src_stride[1] = { static_cast<int>(zed_resized.step[0]) };

        int scaled = sws_scale(
            sws_ctx,
            src_slices,
            src_stride,
            0,
            profile.height,
            video_frame->data,
            video_frame->linesize);

        if (scaled <= 0) {
            RCLCPP_WARN(nodeHandle->get_logger(), "sws_scale failed.");
            return;
        }

        bool force_keyframe =
            force_idr_next.exchange(false, std::memory_order_relaxed) ||
            (frame_pts % profile.keyframe_interval == 0);

        video_frame->pts = frame_pts;

        if (force_keyframe) {
            video_frame->pict_type = AV_PICTURE_TYPE_I;
            video_frame->key_frame = 1;
        } else {
            video_frame->pict_type = AV_PICTURE_TYPE_NONE;
            video_frame->key_frame = 0;
        }

        frame_pts++;

        int ret = avcodec_send_frame(h264_ctx, video_frame);
        if (ret < 0) {
            RCLCPP_WARN(nodeHandle->get_logger(), "H.264 send_frame error: %d", ret);
            return;
        }

        while ((ret = avcodec_receive_packet(h264_ctx, video_packet)) == 0) {
            if (video_packet->size > 0) {
                uint16_t frame_id = static_cast<uint16_t>(frame_pts & 0xFFFF);
                bool ok = send_udp_frame_chunked(server_fd,
                                                 video_packet->data,
                                                 static_cast<size_t>(video_packet->size),
                                                 reinterpret_cast<struct sockaddr*>(&client_addr),
                                                 client_addr_len,
                                                 frame_id);
                if (!ok) {
                    av_packet_unref(video_packet);
                    return;
                }
            }
            av_packet_unref(video_packet);
        }

        if (ret != AVERROR(EAGAIN) && ret != AVERROR_EOF) {
            RCLCPP_WARN(nodeHandle->get_logger(), "H.264 receive_packet error: %d", ret);
        }
    }
}

void zedImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    try {
        cv::Mat gray_mat = cv_bridge::toCvCopy(msg, "mono8")->image;
        if (gray_mat.empty()) return;

        {
            std::lock_guard<std::mutex> lk(img_mutex);
            last_zed_gray = gray_mat.clone();
            last_zed_stamp = msg->header.stamp;
            new_frame_ready = true;
        }
        frame_cv.notify_one();
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(nodeHandle->get_logger(), "zedImageCallback exception: %s", e.what());
    }
}

void encoderWorkerThread() {
    auto last_encode_time = std::chrono::steady_clock::now();

    while (keep_running && rclcpp::ok()) {
        cv::Mat frame_to_encode;

        {
            std::unique_lock<std::mutex> lk(img_mutex);
            frame_cv.wait(lk, [] { return new_frame_ready || !keep_running; });
            if (!keep_running || !rclcpp::ok()) break;

            frame_to_encode = last_zed_gray.clone();
            new_frame_ready = false;
        }

        if (!videoStreaming || server_fd < 0 || !client_connected || frame_to_encode.empty()) {
            continue;
        }

        const StreamProfile profile = get_active_profile();
        const auto frame_interval = std::chrono::milliseconds(1000 / std::max(1, profile.fps));
        const auto now = std::chrono::steady_clock::now();

        if (now - last_encode_time < frame_interval) {
            continue;
        }

        last_encode_time = now;
        encode_and_send(frame_to_encode);
    }
}

std::string getAddressString(int family, std::string interfaceName) {
    std::string addressString("");
    ifaddrs* interfaceAddresses = nullptr;
    int failed = getifaddrs(&interfaceAddresses);
    if (failed || !interfaceAddresses) {
        return addressString;
    }

    for (ifaddrs* it = interfaceAddresses; it; it = it->ifa_next) {
        if (!it->ifa_name || !it->ifa_addr) continue;
        if (strcmp(it->ifa_name, interfaceName.c_str()) != 0) continue;
        if (it->ifa_addr->sa_family != family) continue;

        if (it->ifa_addr->sa_family == AF_INET) {
            sockaddr_in* socketAddress = reinterpret_cast<sockaddr_in*>(it->ifa_addr);
            addressString += inet_ntoa(socketAddress->sin_addr);
        }
        else if (it->ifa_addr->sa_family == AF_INET6) {
            sockaddr_in6* socketAddress = reinterpret_cast<sockaddr_in6*>(it->ifa_addr);
            for (int index = 0; index < 16; index += 2) {
                char bits[5];
                sprintf(bits, "%02x%02x", socketAddress->sin6_addr.s6_addr[index], socketAddress->sin6_addr.s6_addr[index + 1]);
                if (index) addressString += ":";
                addressString += bits;
            }
        }
        else if (it->ifa_addr->sa_family == AF_PACKET) {
            sockaddr_ll* socketAddress = reinterpret_cast<sockaddr_ll*>(it->ifa_addr);
            for (int index = 0; index < socketAddress->sll_halen; index++) {
                char bits[3];
                sprintf(bits, "%02x", socketAddress->sll_addr[index]);
                if (index) addressString += ":";
                addressString += bits;
            }
        }
        break;
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
            memset(&socketAddress, 0, sizeof(socketAddress));
            socketAddress.sin_family = AF_INET;
            socketAddress.sin_addr.s_addr = inet_addr("226.1.1.1");
            socketAddress.sin_port = htons(4322);

            struct in_addr localInterface;
            localInterface.s_addr = inet_addr(addressString.c_str());

            if (setsockopt(socketDescriptor, IPPROTO_IP, IP_MULTICAST_IF,
                           reinterpret_cast<char*>(&localInterface), sizeof(localInterface)) >= 0) {
                sendto(socketDescriptor,
                       message.c_str(),
                       message.length(),
                       0,
                       reinterpret_cast<struct sockaddr*>(&socketAddress),
                       sizeof(socketAddress));
            }
        }
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }

    close(socketDescriptor);
}

static void sendVideoHeartbeat(int sock, const struct sockaddr* dest_addr, socklen_t addrlen) {
    const uint8_t hb[1] = {0};
    ssize_t sent = sendto(sock, hb, sizeof(hb), 0, dest_addr, addrlen);
    if (sent > 0) {
        total_video_bytes_sent.fetch_add(static_cast<uint64_t>(sent), std::memory_order_relaxed);
        last_video_tx_time_ms.store(get_time_ms(), std::memory_order_relaxed);
    }
}

uint64_t get_average_kbps_since(uint64_t start_ms, uint64_t start_bytes) {
    uint64_t now_ms = get_time_ms();
    uint64_t now_bytes = total_video_bytes_sent.load(std::memory_order_relaxed);

    double elapsed_sec = (now_ms > start_ms) ? ((now_ms - start_ms) / 1000.0) : 0.0;
    if (elapsed_sec <= 0.0) return 0;

    uint64_t delta_bytes = now_bytes - start_bytes;
    double kbps = (delta_bytes * 8.0) / 1000.0 / elapsed_sec;
    return static_cast<uint64_t>(kbps);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    nodeHandle = rclcpp::Node::make_shared("video_streaming");
    RCLCPP_INFO(nodeHandle->get_logger(), "Starting video streaming server node");

    std::string zed_topic;

    nodeHandle->declare_parameter<std::string>("interface_name", "wlP1p1s0");
    nodeHandle->declare_parameter<std::string>("robot_name", "shovel");
    nodeHandle->declare_parameter<int>("port", 31338);
    nodeHandle->declare_parameter<std::string>("zed_image_topic", "/zed2i/left/image_raw");
    nodeHandle->declare_parameter<bool>("start_in_clarity_mode", false);

    bool start_in_clarity_mode = false;
    nodeHandle->get_parameter("interface_name", interfaceName);
    nodeHandle->get_parameter("robot_name", robotName);
    nodeHandle->get_parameter("port", clientPort);
    nodeHandle->get_parameter("zed_image_topic", zed_topic);
    nodeHandle->get_parameter("start_in_clarity_mode", start_in_clarity_mode);

    apply_stream_profile(start_in_clarity_mode ? clarity_profile : saver_profile);

    RCLCPP_INFO(nodeHandle->get_logger(), "interface_name: %s", interfaceName.c_str());
    RCLCPP_INFO(nodeHandle->get_logger(), "robot_name: %s", robotName.c_str());
    RCLCPP_INFO(nodeHandle->get_logger(), "port: %d", clientPort);
    RCLCPP_INFO(nodeHandle->get_logger(), "zed_image_topic: %s", zed_topic.c_str());

    auto zed_sub = nodeHandle->create_subscription<sensor_msgs::msg::Image>(
        zed_topic,
        rclcpp::SensorDataQoS(),
        &zedImageCallback);

    image_transport::ImageTransport it(nodeHandle);
    (void)it;
    (void)zed_sub;

    ssize_t bytesRead;
    struct sockaddr_in address;
    int opt = 1;
    uint8_t buffer[2048] = {0};

    if ((server_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        RCLCPP_FATAL(nodeHandle->get_logger(), "UDP Socket creation failed: %s", strerror(errno));
        return EXIT_FAILURE;
    }

    int broadcastEnable = 1;
    if (setsockopt(server_fd, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable)) < 0) {
        RCLCPP_WARN(nodeHandle->get_logger(), "Failed to enable broadcast: %s", strerror(errno));
    }

    int sndbuf = 4 * 1024 * 1024;
    if (setsockopt(server_fd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf)) < 0) {
        RCLCPP_WARN(nodeHandle->get_logger(), "Failed to set send buffer: %s", strerror(errno));
    }

    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt))) {
        RCLCPP_ERROR(nodeHandle->get_logger(), "setsockopt SO_REUSEPORT failed: %s", strerror(errno));
        close(server_fd);
        return EXIT_FAILURE;
    }

    memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(clientPort);

    if (bind(server_fd, reinterpret_cast<struct sockaddr*>(&address), sizeof(address)) < 0) {
        RCLCPP_FATAL(nodeHandle->get_logger(), "Bind failed: %s", strerror(errno));
        close(server_fd);
        return EXIT_FAILURE;
    }
    RCLCPP_INFO(nodeHandle->get_logger(), "Server listening on port %d", clientPort);

    std::thread broadcastThread(broadcastIP);
    std::thread encoderThread(encoderWorkerThread);

    std::list<uint8_t> messageBytesList;
    uint8_t message[256];
    rclcpp::Rate rate(60);

    int flags = fcntl(server_fd, F_GETFL, 0);
    fcntl(server_fd, F_SETFL, flags | O_NONBLOCK);

    auto last_message_time = std::chrono::steady_clock::now();
    uint64_t stats_window_start_ms = get_time_ms();
    uint64_t stats_window_start_bytes = total_video_bytes_sent.load(std::memory_order_relaxed);
    auto last_stats_log = std::chrono::steady_clock::now();

    while (rclcpp::ok()) {
        bytesRead = recvfrom(server_fd,
                             buffer,
                             sizeof(buffer),
                             0,
                             reinterpret_cast<struct sockaddr*>(&client_addr),
                             &client_addr_len);

        if (bytesRead > 0) {
            last_message_time = std::chrono::steady_clock::now();

            std::string received_str(reinterpret_cast<char*>(buffer), bytesRead);
            if (received_str == "Hello Robot") {
                std::string reply("Hello from server");
                sendto(server_fd,
                       reply.c_str(),
                       reply.length(),
                       0,
                       reinterpret_cast<struct sockaddr*>(&client_addr),
                       client_addr_len);

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
                for (ssize_t i = 0; i < bytesRead; i++) {
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
                sendVideoHeartbeat(server_fd, reinterpret_cast<struct sockaddr*>(&client_addr), client_addr_len);
            }
        }

        while (!messageBytesList.empty() && messageBytesList.front() <= messageBytesList.size()) {
            int messageSize = messageBytesList.front();
            messageBytesList.pop_front();
            messageSize--;

            for (int index = 0; index < messageSize; index++) {
                message[index] = messageBytesList.front();
                messageBytesList.pop_front();
            }

            uint8_t command = message[0];

            if (command == 1) {
                videoStreaming = message[1];
                RCLCPP_INFO(nodeHandle->get_logger(), "videoStreaming: %d", videoStreaming);
            }
            else if (command == 2) {
                uint8_t value = message[1];
                (void)value;
            }
            else if (command == 3) {
                force_idr_next.store(true, std::memory_order_relaxed);
            }
            else if (command == 4) {
                uint8_t mode = message[1];
                if (mode == 0) {
                    apply_stream_profile(saver_profile);
                } else {
                    apply_stream_profile(clarity_profile);
                }
            }
            else {
                RCLCPP_DEBUG(nodeHandle->get_logger(), "Unknown command byte: %u", command);
            }
        }

        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_stats_log).count() >= 2) {
            uint64_t avg_kbps = get_average_kbps_since(stats_window_start_ms, stats_window_start_bytes);
            const StreamProfile p = get_active_profile();
            RCLCPP_INFO(nodeHandle->get_logger(),
                        "Video avg bitrate: %lu Kbps | profile=%s | streaming=%d | client=%d",
                        static_cast<unsigned long>(avg_kbps),
                        p.name.c_str(),
                        static_cast<int>(videoStreaming),
                        static_cast<int>(client_connected));
            last_stats_log = now;
        }

        rclcpp::spin_some(nodeHandle);
        rate.sleep();
    }

    RCLCPP_INFO(nodeHandle->get_logger(), "Shutting down video streaming server node.");

    keep_running = false;
    frame_cv.notify_all();
    if (encoderThread.joinable()) {
        encoderThread.join();
    }

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
