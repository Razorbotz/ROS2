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
#include <sensor_msgs/msg/image.hpp>
#include "image_transport/image_transport.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// Jetson Multimedia API
#include "NvVideoEncoder.h"
#include "NvBuffer.h"
#include "NvUtils.h"

#define STREAM_FPS 30
#define STREAM_HEIGHT 400
#define LEFT_WIDTH 640
#define RIGHT_WIDTH 640
#define STITCHED_WIDTH (LEFT_WIDTH + RIGHT_WIDTH)
#define BITRATE 4000000 // 4 Mbps CBR
#define IDR_INTERVAL 10
#define PORT 31338

// Global state
rclcpp::Node::SharedPtr nodeHandle;
bool broadcast = true;
bool videoStreaming = false;
bool client_connected = false;
int server_fd = -1;

struct sockaddr_in client_addr;
socklen_t client_addr_len = sizeof(client_addr);

NvVideoEncoder* g_encoder = nullptr;
std::mutex encoder_mutex;

// Frame storage
std::mutex img_mutex;
cv::Mat last_zed_gray, last_rs_gray;

// UDP chunk header (same as your client)
struct FrameHeader {
    uint16_t frame_id;
    uint16_t chunk_index;
    uint16_t total_chunks;
} __attribute__((packed));

bool init_nvencoder(int width, int height)
{
    std::lock_guard<std::mutex> lock(encoder_mutex);

    if (g_encoder) return true;

    RCLCPP_INFO(nodeHandle->get_logger(), "[NvEnc] Initializing H.265 encoder %dx%d", width, height);

    g_encoder = NvVideoEncoder::createVideoEncoder("enc0");
    if (!g_encoder) {
        RCLCPP_ERROR(nodeHandle->get_logger(), "[NvEnc] createVideoEncoder failed");
        return false;
    }

    if (g_encoder->setCapturePlaneFormat(V4L2_PIX_FMT_HEVC, width, height, 2 * 1024 * 1024) < 0 ||
        g_encoder->setOutputPlaneFormat(V4L2_PIX_FMT_YUV420M, width, height) < 0)
    {
        RCLCPP_ERROR(nodeHandle->get_logger(), "[NvEnc] setFormat failed");
        return false;
    }

    g_encoder->setBitrate(BITRATE);
    g_encoder->setRateControlMode(V4L2_MPEG_VIDEO_BITRATE_MODE_CBR);
    g_encoder->setFrameRate(STREAM_FPS, 1);
    g_encoder->setProfile(V4L2_MPEG_VIDEO_H265_PROFILE_MAIN);
    g_encoder->setIDRInterval(IDR_INTERVAL);
    g_encoder->setInsertSpsPpsAtIdrEnabled(true);

    g_encoder->output_plane.setupPlane(V4L2_MEMORY_USERPTR, 6, true, false);
    g_encoder->capture_plane.setupPlane(V4L2_MEMORY_MMAP, 6, true, false);

    g_encoder->output_plane.setStreamStatus(true);
    g_encoder->capture_plane.setStreamStatus(true);

    RCLCPP_INFO(nodeHandle->get_logger(), "[NvEnc] Ready");
    return true;
}

void encode_and_send(const cv::Mat& gray)
{
    if (!g_encoder || !videoStreaming) return;

    std::lock_guard<std::mutex> lock(encoder_mutex);

    cv::Mat yuv;
    cv::cvtColor(gray, yuv, cv::COLOR_GRAY2BGR);
    cv::cvtColor(yuv, yuv, cv::COLOR_BGR2YUV_I420);

    struct v4l2_buffer v4l2_buf {};
    struct v4l2_plane planes[MAX_PLANES] {};

    v4l2_buf.type = g_encoder->output_plane.getBufType();
    v4l2_buf.memory = V4L2_MEMORY_USERPTR;
    v4l2_buf.m.planes = planes;
    v4l2_buf.m.planes[0].m.userptr = (unsigned long)yuv.data;
    v4l2_buf.m.planes[0].bytesused = yuv.total();
    v4l2_buf.m.planes[0].length = yuv.total();

    if (g_encoder->output_plane.qBuffer(v4l2_buf, nullptr) < 0) return;

    struct v4l2_buffer enc_buf {};
    struct v4l2_plane enc_planes[MAX_PLANES] {};

    enc_buf.type = g_encoder->capture_plane.getBufType();
    enc_buf.memory = V4L2_MEMORY_MMAP;
    enc_buf.m.planes = enc_planes;

    NvBuffer* buffer = nullptr;
    NvBuffer* shared_buffer = nullptr;
    uint32_t bytes_used = 0;

    if (g_encoder->capture_plane.dqBuffer(enc_buf,
                                        &buffer,
                                        &shared_buffer,
                                        bytes_used) < 0)
    {
        RCLCPP_WARN(nodeHandle->get_logger(), "[NvEnc] dqBuffer failed");
        return;
    }

    uint8_t* enc_data =
        g_encoder->capture_plane.getNthBuffer(enc_buf.index)->planes[0].data;
    size_t enc_size =
        enc_buf.m.planes[0].bytesused;

    // Send via UDP in chunks (same protocol)
    const size_t CHUNK = 1300;
    uint16_t frame_id = (uint16_t)(std::chrono::steady_clock::now().time_since_epoch().count());
    uint16_t total = (enc_size + CHUNK - 1) / CHUNK;

    for (uint16_t i = 0; i < total; i++) {
        size_t off = i * CHUNK;
        size_t len = std::min(CHUNK, enc_size - off);

        FrameHeader hdr {
            htons(frame_id),
            htons(i),
            htons(total)
        };

        uint8_t pkt[sizeof(hdr) + CHUNK];
        memcpy(pkt, &hdr, sizeof(hdr));
        memcpy(pkt + sizeof(hdr), enc_data + off, len);

        sendto(server_fd, pkt, sizeof(hdr) + len, 0,
               (struct sockaddr*)&client_addr, client_addr_len);
    }

    g_encoder->capture_plane.qBuffer(enc_buf, buffer);
}

void stitch_and_encode()
{
    if (!videoStreaming || !client_connected || server_fd < 0) return;

    cv::Mat left, right;
    {
        std::lock_guard<std::mutex> lk(img_mutex);
        if (last_zed_gray.empty() || last_rs_gray.empty()) return;
        left = last_zed_gray.clone();
        right = last_rs_gray.clone();
    }

    cv::Mat l_resized, r_resized, stitched;
    cv::resize(left, l_resized, cv::Size(LEFT_WIDTH, STREAM_HEIGHT));
    cv::resize(right, r_resized, cv::Size(RIGHT_WIDTH, STREAM_HEIGHT));
    cv::hconcat(l_resized, r_resized, stitched);

    if (!g_encoder && !init_nvencoder(STITCHED_WIDTH, STREAM_HEIGHT)) return;

    encode_and_send(stitched);
}

void zedImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg){
    cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;
    cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
    {
        std::lock_guard<std::mutex> lk(img_mutex);
        last_zed_gray = img.clone();
    }
    stitch_and_encode();
}

void intelImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg){
    cv::Mat img = cv_bridge::toCvCopy(msg, "rgb8")->image;
    cv::cvtColor(img, img, cv::COLOR_RGB2GRAY);
    {
        std::lock_guard<std::mutex> lk(img_mutex);
        last_rs_gray = img.clone();
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

    int broadcastEnable = 1;
    if (setsockopt(server_fd, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable)) < 0) {
        RCLCPP_WARN(nodeHandle->get_logger(), "Failed to enable broadcast: %s", strerror(errno));
    }

    // Reduce kernel buffering latency
    int sndbuf = 1 * 1024 * 1024;
    if (setsockopt(server_fd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf)) < 0) {
        RCLCPP_WARN(nodeHandle->get_logger(), "Failed to set send buffer: %s", strerror(errno));
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