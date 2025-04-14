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

#define PORT 31338

bool videoStreaming = false;
int new_socket = -1;
rclcpp::Node::SharedPtr nodeHandle;
bool broadcast = true;
// cv::Mat img;
cv::Mat gray;
bool isGray = true;

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
                RCLCPP_INFO(nodeHandle->get_logger(), "send interrupted by EINTR, retrying.");
                continue;
            }
            else if (errno == EAGAIN || errno == EWOULDBLOCK) {
                RCLCPP_INFO(nodeHandle->get_logger(), "send temporarily unavailable, retrying.");
                std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Add a small delay before retrying
                continue;
            }
            else {
                RCLCPP_ERROR(nodeHandle->get_logger(), "send failed: %s", strerror(errno));
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
void zedImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & inputImage){
    if(videoStreaming && new_socket >= 0){
        cv::Mat frame_to_send;
        try {
             cv::Mat img_color = cv_bridge::toCvCopy(inputImage, "rgb8")->image;

             if(isGray){
                 cv::cvtColor(img_color, gray, cv::COLOR_RGB2GRAY);
                 frame_to_send = gray;
             }
             else{
                 frame_to_send = img_color;
             }

             if (frame_to_send.empty()) {
                 RCLCPP_WARN(nodeHandle->get_logger(), "Frame to send is empty after conversion.");
                 return;
             }
             if (!frame_to_send.isContinuous()) {
                 frame_to_send = frame_to_send.clone();
             }

             uint32_t frame_size = frame_to_send.total() * frame_to_send.elemSize();
             uint32_t network_frame_size = htonl(frame_size);

             if (!send_all(new_socket, &network_frame_size, sizeof(network_frame_size))) {
                 RCLCPP_ERROR(nodeHandle->get_logger(), "Failed to send frame size header. Stopping stream.");
                 videoStreaming = false;
                 return;
             }

             if (!send_all(new_socket, frame_to_send.data, frame_size)) {
                 RCLCPP_ERROR(nodeHandle->get_logger(), "Failed to send frame data. Stopping stream.");
                 videoStreaming = false;
                 return;
             }

         }
         catch (cv_bridge::Exception& e) {
             RCLCPP_ERROR(nodeHandle->get_logger(), "cv_bridge exception: %s", e.what());
             videoStreaming = false;
             return;
         }
         catch (cv::Exception& e) {
             RCLCPP_ERROR(nodeHandle->get_logger(), "OpenCV exception: %s", e.what());
             videoStreaming = false;
             return;
         }
         catch (const std::exception& e) {
             RCLCPP_ERROR(nodeHandle->get_logger(), "Standard exception: %s", e.what());
             videoStreaming = false;
             return;
         }
         catch (...) {
             RCLCPP_ERROR(nodeHandle->get_logger(), "Unknown exception occurred in zedImageCallback.");
             videoStreaming = false;
             return;
         }
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
        RCLCPP_INFO(nodeHandle->get_logger(),"bytes read %ld", bytesRead);
        RCLCPP_INFO(nodeHandle->get_logger(), "messageBytesList.size(): %ld", messageBytesList.size());
        RCLCPP_INFO(nodeHandle->get_logger(), "messageBytesList.front(): %d", messageBytesList.front());
        
        while(messageBytesList.size()>0 && messageBytesList.front()<=messageBytesList.size()){
            RCLCPP_INFO(nodeHandle->get_logger(),"bytes read %ld", bytesRead);
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