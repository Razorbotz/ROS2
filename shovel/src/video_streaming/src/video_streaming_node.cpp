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
#include <linux/reboot.h>
#include <sys/reboot.h>

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

bool videoStreaming=false;
int new_socket;
rclcpp::Node::SharedPtr nodeHandle;
int total = 0;
bool broadcast = true;
cv::Mat img;
cv::Mat gray;
bool isGray = true;

/** @brief Receives the ZED camera image
 * 
 * This function hasn't been fully implemented yet.  In the future, this
 * will send the received image to the client-side GUI.
 * @param inputImage 
 */
void zedImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & inputImage){
    if(videoStreaming){
        img = cv_bridge::toCvCopy(inputImage, "rgb8")->image;
         if(isGray){
            cv::cvtColor(img, gray, CV_RGB2GRAY);
            int imgSize = gray.total()*gray.elemSize();
            int bytes = 0, total = 0;
            while(total < imgSize){
                bytes = send(new_socket, gray.data, imgSize - total, 0);
                if(bytes != -1){
                    total += bytes;
                }
            }
        }
        else{
            int imgSize = img.total()*img.elemSize();
            int bytes = 0, total = 0;
            while(total < imgSize){
                bytes = send(new_socket, img.data, imgSize - total, 0);
                if(bytes != -1){
                    total += bytes;
                }
            }
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
    RCLCPP_INFO(nodeHandle->get_logger(),"Starting video streaming node");

    image_transport::ImageTransport it(nodeHandle);
    image_transport::Subscriber sub = it.subscribe("zed_image", 1, zedImageCallback);

    int server_fd, bytesRead; 
    struct sockaddr_in address; 
    int opt = 1; 
    int addrlen = sizeof(address); 
    uint8_t buffer[2048] = {0}; 
    std::string hello("Hello from server");


    // Creating socket file descriptor
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) { 
        perror("socket failed"); 
        exit(EXIT_FAILURE); 
    }
    std::thread broadcastThread(broadcastIP);

    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) { 
        perror("setsockopt"); 
        exit(EXIT_FAILURE); 
    } 
    address.sin_family = AF_INET; 
    address.sin_addr.s_addr = INADDR_ANY; 
    address.sin_port = htons( PORT ); 

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address))<0) { 
        perror("bind failed"); 
        exit(EXIT_FAILURE); 
    } 
    if (listen(server_fd, 3) < 0) { 
        perror("listen"); 
        exit(EXIT_FAILURE); 
    } 
    if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen))<0) { 
        perror("accept"); 
        exit(EXIT_FAILURE); 
    }

    bytesRead = read(new_socket, buffer, 2048); 
    send(new_socket, hello.c_str(), strlen(hello.c_str()), 0); 

    fcntl(new_socket, F_SETFL, O_NONBLOCK);
    

    std::list<uint8_t> messageBytesList;
    uint8_t message[256];
    rclcpp::Rate rate(20);
    while(rclcpp::ok()){
        bytesRead = recv(new_socket, buffer, 2048, 0);
        for(int index=0;index<bytesRead;index++){
            messageBytesList.push_back(buffer[index]);
        }

        if(bytesRead==0){
	        RCLCPP_INFO(nodeHandle->get_logger(),"Lost Connection");
            videoStreaming = false;
            broadcast=true;
            //wait for reconnect
            if (listen(server_fd, 3) < 0) { 
                perror("listen"); 
                exit(EXIT_FAILURE); 
            } 
            if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen))<0) { 
                perror("accept"); 
                exit(EXIT_FAILURE); 
            }
            broadcast=false;
            bytesRead = read(new_socket, buffer, 2048); 
            send(new_socket, hello.c_str(), strlen(hello.c_str()), 0); 
            fcntl(new_socket, F_SETFL, O_NONBLOCK);
        }
        while(messageBytesList.size()>0 && messageBytesList.front()<=messageBytesList.size()){
	    //RCLCPP_INFO(nodeHandle->get_logger(),"bytes read %d", bytesRead);
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

    broadcastThread.join();

}
