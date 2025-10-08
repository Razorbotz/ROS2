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
int new_socket = -1;
rclcpp::Node::SharedPtr nodeHandle;
bool broadcast = true;
// cv::Mat img;
cv::Mat gray;
bool isGray = true;
int counter = 0;


/**
 * @brief Initializes the H.265 encoder. Call this when video streaming begins.
 * @param width The width of the video frames to be encoded.
 * @param height The height of the video frames to be encoded.
 * @return True on success, false on failure.
 */
bool initialize_h265_encoder(int width, int height) {
    const AVCodec* codec = avcodec_find_encoder_by_name("libx265");
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


/**
 * @brief Sends all data specified in the buffer over the socket.
 * Handles partial sends and retries on EINTR. Assumes blocking socket.
 * Uses MSG_NOSIGNAL to prevent SIGPIPE from crashing the server.
 * @param sock The socket descriptor.
 * @param data Pointer to the data buffer.
 * @param len The number of bytes to send.
 * @return true if all data was sent successfully, false on error or disconnect.
 */
bool send_all(int sock, const void* data, size_t len) {
    const char* ptr = static_cast<const char*>(data);
    size_t total_sent = 0;
    while (total_sent < len) {
        ssize_t bytes_sent = send(sock, ptr + total_sent, len - total_sent, MSG_NOSIGNAL);

        if (bytes_sent > 0) {
            total_sent += bytes_sent;
        }
        else if (bytes_sent == 0) {
            RCLCPP_ERROR(nodeHandle->get_logger(), "send returned 0 unexpectedly.");
            return false;
        }
        else {
            if (errno == EINTR) {
                //RCLCPP_INFO(nodeHandle->get_logger(), "send interrupted by EINTR, retrying.");
                continue;
            }
            else if (errno == EAGAIN || errno == EWOULDBLOCK) {
                //RCLCPP_INFO(nodeHandle->get_logger(), "send temporarily unavailable, retrying.");
                std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Add a small delay before retrying
                continue;
            }
            else {
                //RCLCPP_ERROR(nodeHandle->get_logger(), "send failed: %s", strerror(errno));
                return false;
            }
        }
    }
    return true;
}


/** @brief Receives the ZED camera image and sends it to the client
 *
 * This function converts the received ROS image message, optionally converts
 * it to grayscale, and sends it over the TCP socket to the connected client
 * using a framing protocol (4-byte size header + raw data).
 * @param inputImage The ROS image message.
 */
void zedImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & inputImage) {
    // --- Define resolution in one place ---
    const int STREAM_WIDTH = 640;
    const int STREAM_HEIGHT = 400;

    // If we aren't streaming, ensure everything is cleaned up and exit.
    if (!videoStreaming || new_socket < 0) {
        if (h265_encoder_ctx) {
            cleanup_h265_encoder();
        }
        return;
    }
    
    try {
        cv::Mat img_bgr = cv_bridge::toCvCopy(inputImage, "bgr8")->image;
        if (img_bgr.empty()) {
            RCLCPP_WARN(nodeHandle->get_logger(), "Received empty image frame.");
            return;
        }

        // --- Synchronized Initialization Block ---
        // If the main encoder context doesn't exist, we need to set up the entire pipeline.
        if (!h265_encoder_ctx) {
            RCLCPP_INFO(nodeHandle->get_logger(), "Initializing H.265 pipeline for %dx%d.", STREAM_WIDTH, STREAM_HEIGHT);
            
            // 1. Initialize the encoder
            if (!initialize_h265_encoder(STREAM_WIDTH, STREAM_HEIGHT)) {
                RCLCPP_ERROR(nodeHandle->get_logger(), "Failed to initialize H.265 encoder. Halting stream.");
                videoStreaming = false;
                cleanup_h265_encoder(); // Ensure partial initializations are cleaned
                return;
            }

            // 2. Initialize the color converter context right after, using the same dimensions.
            sws_ctx = sws_getContext(STREAM_WIDTH, STREAM_HEIGHT, AV_PIX_FMT_BGR24,
                                     STREAM_WIDTH, STREAM_HEIGHT, h265_encoder_ctx->pix_fmt,
                                     SWS_BILINEAR, nullptr, nullptr, nullptr);
            if (!sws_ctx) {
                RCLCPP_ERROR(nodeHandle->get_logger(), "Failed to create SWS context. Halting stream.");
                videoStreaming = false;
                cleanup_h265_encoder(); // sws_ctx is also cleaned up here
                return;
            }
        }
        // --- End of Initialization Block ---

        cv::Mat resized_frame;
        cv::resize(img_bgr, resized_frame, cv::Size(STREAM_WIDTH, STREAM_HEIGHT), 0, 0, cv::INTER_AREA);

        // Convert the resized BGR frame to YUV420P for the encoder
        const int stride[] = { static_cast<int>(resized_frame.step[0]) };
        sws_scale(sws_ctx, &resized_frame.data, stride, 0, resized_frame.rows,
                  video_frame->data, video_frame->linesize);

        video_frame->pts = frame_pts++;

        // Send the raw frame to the encoder
        if (avcodec_send_frame(h265_encoder_ctx, video_frame) < 0) {
            RCLCPP_WARN(nodeHandle->get_logger(), "Error sending a frame to the H.265 encoder.");
            return;
        }

        // Receive any encoded packets and send them over the network
        while (avcodec_receive_packet(h265_encoder_ctx, video_packet) == 0) {
            size_t encoded_size = video_packet->size;
            if (encoded_size == 0) continue; // Skip empty packets

            uint32_t network_frame_size = htonl(encoded_size);

            if (!send_all(new_socket, &network_frame_size, sizeof(network_frame_size)) || 
                !send_all(new_socket, video_packet->data, encoded_size)) 
            {
                RCLCPP_ERROR(nodeHandle->get_logger(), "Failed to send packet, client disconnected.");
                videoStreaming = false;
                close(new_socket);
                new_socket = -1;
                av_packet_unref(video_packet);
                return; // Exit callback on send failure
            }
            av_packet_unref(video_packet);
        }

    } catch (const std::exception& e) {
        RCLCPP_ERROR(nodeHandle->get_logger(), "Exception in zedImageCallback: %s", e.what());
        videoStreaming = false;
        return;
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
void broadcastIP(){
    while(true){
        if(broadcast){
            std::string addressString=getAddressString(AF_INET,"wlan0");

            std::string message(robotName+"@"+addressString);
            std::cout << message << std::endl << std::flush;

            int socketDescriptor=socket(AF_INET, SOCK_DGRAM, 0);

            //if(socket>=0){
            if(socketDescriptor>=0){
                struct sockaddr_in socketAddress;
                socketAddress.sin_family=AF_INET;
                socketAddress.sin_addr.s_addr = inet_addr("226.1.1.1");
                socketAddress.sin_port = htons(4322);

                struct in_addr localInterface;
                localInterface.s_addr = inet_addr(addressString.c_str());
                if(setsockopt(socketDescriptor, IPPROTO_IP, IP_MULTICAST_IF, (char*)&localInterface, sizeof(localInterface))>=0){
                    sendto(socketDescriptor,message.c_str(),message.length(),0,(struct sockaddr*)&socketAddress, sizeof(socketAddress));
                }
            }
            close(socketDescriptor);
        }
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }
}


int main(int argc, char **argv){
    rclcpp::init(argc,argv);

    nodeHandle = rclcpp::Node::make_shared("video_streaming");
    RCLCPP_INFO(nodeHandle->get_logger(),"Starting video streaming server node");

    image_transport::ImageTransport it(nodeHandle);
    image_transport::Subscriber sub = it.subscribe("zed_image", 1, zedImageCallback);

    int server_fd = -1;
    ssize_t bytesRead;
    struct sockaddr_in address;
    int opt = 1;
    socklen_t addrlen = sizeof(address);
    uint8_t buffer[2048] = {0};
    std::string hello("Hello from server");

    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        RCLCPP_FATAL(nodeHandle->get_logger(), "Socket creation failed: %s", strerror(errno));
        return EXIT_FAILURE;
    }

    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
        RCLCPP_ERROR(nodeHandle->get_logger(), "setsockopt failed: %s", strerror(errno));
        close(server_fd);
        return EXIT_FAILURE;
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons( PORT );

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address))<0) {
        RCLCPP_FATAL(nodeHandle->get_logger(), "Bind failed: %s", strerror(errno));
        close(server_fd);
        return EXIT_FAILURE;
    }
    if (listen(server_fd, 3) < 0) {
        RCLCPP_FATAL(nodeHandle->get_logger(), "Listen failed: %s", strerror(errno));
        close(server_fd);
        return EXIT_FAILURE;
    }
    RCLCPP_INFO(nodeHandle->get_logger(), "Server listening on port %d", PORT);

    std::thread broadcastThread(broadcastIP);

    std::list<uint8_t> messageBytesList;
    uint8_t message[256];
    rclcpp::Rate rate(20);
    while(rclcpp::ok()){
        if (new_socket < 0) {
            RCLCPP_INFO(nodeHandle->get_logger(), "Waiting for client connection...");
            videoStreaming = false;
            broadcast = true;
            new_socket = accept(server_fd, (struct sockaddr *)&address, &addrlen);

            if (new_socket < 0) {
                RCLCPP_ERROR(nodeHandle->get_logger(), "Accept failed: %s", strerror(errno));
                if (errno == EBADF || errno == EINVAL) {
                    RCLCPP_FATAL(nodeHandle->get_logger(), "Server socket invalid state, exiting.");
                    break;
                }
                rate.sleep();
                continue;
            }

            char client_ip[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &address.sin_addr, client_ip, INET_ADDRSTRLEN);
            RCLCPP_INFO(nodeHandle->get_logger(), "Client connected from %s", client_ip);
            broadcast = false;
            bytesRead = read(new_socket, buffer, 2048); 

            if (!send_all(new_socket, hello.c_str(), hello.length())) {
                RCLCPP_ERROR(nodeHandle->get_logger(), "Failed to send hello to client.");
                close(new_socket);
                new_socket = -1;
                continue;
            }

            int flags = fcntl(new_socket, F_GETFL, 0);
            if (flags == -1 || fcntl(new_socket, F_SETFL, flags | O_NONBLOCK) == -1) {
                RCLCPP_ERROR(nodeHandle->get_logger(), "Failed to set client socket non-blocking: %s", strerror(errno));
                close(new_socket);
                new_socket = -1;
                continue;
            }
        }

        try{
            bytesRead = recv(new_socket, buffer, sizeof(buffer), 0);

            if (bytesRead > 0) {
                for(int index = 0; index < bytesRead; index++) {
                    messageBytesList.push_back(buffer[index]);
                }
            } 
            else if (bytesRead == 0) {
                RCLCPP_INFO(nodeHandle->get_logger(), "Client disconnected gracefully.");
                close(new_socket);
                new_socket = -1;
                videoStreaming = false;
                messageBytesList.clear();
                continue;
            }
            else {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                }
                else {
                    RCLCPP_ERROR(nodeHandle->get_logger(), "recv failed: %s", strerror(errno));
                    close(new_socket);
                    new_socket = -1;
                    videoStreaming = false;
                    messageBytesList.clear();
                    continue;
                }
            }
        
        }
        catch(int x){
            RCLCPP_INFO(nodeHandle->get_logger(), "ERROR: Exception when trying to read data from client");
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
                if(value % 2 == 0){
                    isGray = true;
                }
                else{
                    isGray = false;
                }
            }
        }

        rclcpp::spin_some(nodeHandle);
        rate.sleep();
    }

    RCLCPP_INFO(nodeHandle->get_logger(), "Shutting down video streaming server node.");
    if (new_socket >= 0) {
        close(new_socket);
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