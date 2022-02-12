#include <unistd.h>
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
#include <linux/reboot.h>
#include <sys/reboot.h>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>

#include <messages/msg/power.hpp>
#include <messages/msg/key_state.hpp>
#include <messages/msg/hat_state.hpp>  
#include <messages/msg/button_state.hpp>
#include <messages/msg/axis_state.hpp>
#include <messages/msg/talon_out.hpp>
#include <messages/msg/victor_out.hpp>

#define PORT 31337

/** @file
 * @brief Node for handling communication between the client and the rover.
 * Detailed description of file 
 *  
 * This node receives information published by the power_distribution_panel node, motor nodes, and the logic node
 * wraps the information into topics, then publishes the topics.  
 * Currently this node is missing the callback functions for the motors. 
 * The topics that the node subscribes to are as follows:
 * \li \b power
 * \li \b rev motor topic 
 * \li \b drive_state
 * \li \b STOP 
 * \li \b GO
 * 
 * The topics that are being published are as follows:
 * \li \b joystick_axis
 * \li \b joystick_button
 * \li \b joystick_hat
 * \li \b key
 * 
 * To read more about the nodes that subscribe to this one
 * \see logic_node.cpp
 * 
 * 
 * */

std_msgs::msg::Empty empty;
bool silentRunning=true;

/** @brief Inserts topic into a payload to be sent.
 * Takes a float topic and an array and stores a byte represenation of the topic.
 * @param array
 * @return void
 * */
void insert(float value,uint8_t* array){
    array[0]=uint8_t((uint32_t(*(static_cast<uint32_t*>(static_cast<void*>(&value))))>>24) & 0xff);
    array[1]=uint8_t((uint32_t(*(static_cast<uint32_t*>(static_cast<void*>(&value))))>>16) & 0xff);
    array[2]=uint8_t((uint32_t(*(static_cast<uint32_t*>(static_cast<void*>(&value))))>>8) & 0xff);
    array[3]=uint8_t((uint32_t(*(static_cast<uint32_t*>(static_cast<void*>(&value))))>>0) & 0xff);
}

/** @brief Inserts topic into a payload to be sent.
 * Takes an int topic and an array and stores a byte represenation of the topic.
 * @param array
 * @return void
 * */
void insert(int value,uint8_t* array){
    array[0]=uint8_t((uint32_t(*(static_cast<uint32_t*>(static_cast<void*>(&value))))>>24) & 0xff);
    array[1]=uint8_t((uint32_t(*(static_cast<uint32_t*>(static_cast<void*>(&value))))>>16) & 0xff);
    array[2]=uint8_t((uint32_t(*(static_cast<uint32_t*>(static_cast<void*>(&value))))>>8) & 0xff);
    array[3]=uint8_t((uint32_t(*(static_cast<uint32_t*>(static_cast<void*>(&value))))>>0) & 0xff);
}

/** @brief Parse a byte represenation into a float.
 * 
 * @param array
 * @return value
 * */
float parseFloat(uint8_t* array){
    uint32_t axisYInteger=0;
	// bit wise shifts of the array
    axisYInteger|=uint32_t(array[0])<<24;    
    axisYInteger|=uint32_t(array[1])<<16;    
    axisYInteger|=uint32_t(array[2])<<8;    
    axisYInteger|=uint32_t(array[3])<<0;    
    float value=(float)*(static_cast<float*>(static_cast<void*>(&axisYInteger)));

    return value;
}

/** @brief Parse a byte represenation into an int.
 * 
 * @param array
 * @return value
 * */
 
int parseInt(uint8_t* array){
    uint32_t axisYInteger=0;
    axisYInteger|=uint32_t(array[0])<<24;    
    axisYInteger|=uint32_t(array[1])<<16;    
    axisYInteger|=uint32_t(array[2])<<8;    
    axisYInteger|=uint32_t(array[3])<<0;    
    int value=(int)*(static_cast<int*>(static_cast<void*>(&axisYInteger)));

    return value;
}

 
int new_socket;

 
/** @brief Callback function for the power topic.
 * 
 * This function is called when the node receives a
 * topic with the name power. This function
 * extracts the information given from the power topic
 * and places data into a payload to be sent.
 *  
 * \see .power_distribution_panel.cpp
 * @param power
 * @return void
 * */
void powerCallback(const messages::msg::Power::SharedPtr power){
//    std::cout << "power " << power->voltage << std::endl;
    if(silentRunning)return;
    int messageSize=17*4+2;
    uint8_t message[17*4+2];
    message[0]=messageSize;
    message[1]=1;
	// floatPointer points to the location in memory of the third location of the message var.
    // A void pointer can point to a variable of any data type.
	float* floatPointer=(float*)(void*)&message[2];

	// values from power.msg stored into floatPointer
    insert(power->voltage,(uint8_t*)floatPointer++);
    insert(power->current0,(uint8_t*)floatPointer++);
    insert(power->current1,(uint8_t*)floatPointer++);
    insert(power->current2,(uint8_t*)floatPointer++);
    insert(power->current3,(uint8_t*)floatPointer++);
    insert(power->current4,(uint8_t*)floatPointer++);
    insert(power->current5,(uint8_t*)floatPointer++);
    insert(power->current6,(uint8_t*)floatPointer++);
    insert(power->current7,(uint8_t*)floatPointer++);
    insert(power->current8,(uint8_t*)floatPointer++);
    insert(power->current9,(uint8_t*)floatPointer++);
    insert(power->current10,(uint8_t*)floatPointer++);
    insert(power->current11,(uint8_t*)floatPointer++);
    insert(power->current12,(uint8_t*)floatPointer++);
    insert(power->current13,(uint8_t*)floatPointer++);
    insert(power->current14,(uint8_t*)floatPointer++);
    insert(power->current15,(uint8_t*)floatPointer++);
    
    send(new_socket, message, messageSize, 0);
}


uint8_t driveState=1;
/** @brief Callback function for the driveState topic.
 *
 * This function is called when the node receives a
 * topic with the name driveState. This function
 * extracts the information given from the driveState topic
 * and places data into a payload to be sent.
 * @param state
 * @return void
 * */
void driveStateCallback(const std_msgs::msg::Bool::SharedPtr state){
    int messageSize=3;
    uint8_t message[3];
    message[0]=messageSize;
    message[1]=7;
    message[2]=(uint8_t)state->data;
    driveState=(uint8_t)state->data;
    
    send(new_socket, message, messageSize, 0);
}

/** @brief Function for sending the current state of the rover.
 * 
 * This function is called when the node receives a
 * request for the rovers current state. This function
 * places the driveState var into a payload to be sent.
 * */
void sendRobotState() {
    int messageSize = 3;
    uint8_t message[3];
    message[0] = messageSize;
    message[1] = 7;
    message[2] = driveState;

    send(new_socket, message, messageSize, 0);
}

/** @brief Returns the address string of the rover.
 * 
 * This function is called when the node
 * tries to setup the socket connection between the rover and client. This function
 * returns the address as a string.
 * @param family
 * @param interfaceName
 * @return addressString
 * */
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


/** @brief Prints the address
 * 
 * */
void printAddresses() {
    printf("Addresses\n");
    ifaddrs* interfaceAddresses = nullptr;
    for (int failed=getifaddrs(&interfaceAddresses); !failed && interfaceAddresses; interfaceAddresses=interfaceAddresses->ifa_next){
        printf("%s ",interfaceAddresses->ifa_name);
        if(interfaceAddresses->ifa_addr->sa_family == AF_INET){
            printf("AF_INET ");
            sockaddr_in* socketAddress=reinterpret_cast<sockaddr_in*>(interfaceAddresses->ifa_addr);
            printf("%d ",socketAddress->sin_port);
            printf("%s ",inet_ntoa(socketAddress->sin_addr));
        }
        if(interfaceAddresses->ifa_addr->sa_family == AF_INET6){
            printf("AF_INET6 ");
            sockaddr_in6* socketAddress=reinterpret_cast<sockaddr_in6*>(interfaceAddresses->ifa_addr);
            printf("%d ",socketAddress->sin6_port);
            printf("%d ",socketAddress->sin6_flowinfo); 
            for(int index=0;index<16;index+=2) {
                if(index)printf(":");
                printf("%02x%02x",socketAddress->sin6_addr.s6_addr[index],socketAddress->sin6_addr.s6_addr[index+1]);
            }
        }
        if(interfaceAddresses->ifa_addr->sa_family == AF_PACKET){
            printf("AF_PACKET ");
            sockaddr_ll* socketAddress=reinterpret_cast<sockaddr_ll*>(interfaceAddresses->ifa_addr);
            printf("%d ",socketAddress->sll_protocol);
            printf("%d ",socketAddress->sll_ifindex);
            printf("%d ",socketAddress->sll_hatype);
            printf("%d ",socketAddress->sll_pkttype);
            for(int index=0;index<socketAddress->sll_halen;index++){
                if(index)printf(":");
                printf("%02x",socketAddress->sll_addr[index]);
            }
        }
        printf("\n");
    }
    printf("Done\n");
}

/** @brief Reboots the rover. 
 *
 * */
void reboot(){
    sync();
    reboot(LINUX_REBOOT_CMD_POWER_OFF);
}

std::string robotName="unnamed";
bool broadcast=true;
/** @brief Creates socketDescriptor for socket connection.
 * 
 * This function is called when the node
 * tries to setup the socket connection between the rover and client.
 * This function creates the socketDescriptor for the socket connection.
 * Uses the getAddressString function.
 * */
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
                socketAddress.sin_port = htons(4321);

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
	// initialize arguments 
    rclcpp::init(argc,argv);

	// create a new subnode 
    rclcpp::Node::SharedPtr nodeHandle = rclcpp::Node::make_shared("communication");
    RCLCPP_INFO(nodeHandle->get_logger(),"Starting communication node");

	// set robots name
    nodeHandle->declare_parameter<std::string>("robot_name","not named");
    rclcpp::Parameter robotNameParameter = nodeHandle->get_parameter("robot_name");
    robotName = robotNameParameter.as_string();
    RCLCPP_INFO(nodeHandle->get_logger(),"robotName: %s", robotName.c_str());

	// create publisher subnodes
    auto joystickAxisPublisher = nodeHandle->create_publisher<messages::msg::AxisState>("joystick_axis", 1);
    auto joystickHatPublisher = nodeHandle->create_publisher<messages::msg::HatState>("joystick_hat",1);
    auto joystickButtonPublisher = nodeHandle->create_publisher<messages::msg::ButtonState>("joystick_button",1);
    auto keyPublisher = nodeHandle->create_publisher<messages::msg::KeyState>("key",1);
    auto stopPublisher = nodeHandle->create_publisher<std_msgs::msg::Empty>("STOP",1);
    auto goPublisher=nodeHandle->create_publisher<std_msgs::msg::Empty>("GO",1);

	// create subscriber subnodes and set callbacks 
    auto powerSubscriber = nodeHandle->create_subscription<messages::msg::Power>("power",1,powerCallback);
    auto driveStateSubscriber = nodeHandle->create_subscription<std_msgs::msg::Bool>("drive_state",1,driveStateCallback);
     
	// initialize vars
    int server_fd, bytesRead; 
    struct sockaddr_in address; 
    int opt = 1; 
    int addrlen = sizeof(address); 
    uint8_t buffer[1024] = {0}; 
    std::string hello("Hello from server");

    float state0 = 0.0;
    float state1 = 0.0;
    float state2 = 0.0;

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
	// initialize vars for connecting the sockets
    broadcast=false;
    bytesRead = read(new_socket, buffer, 1024); 
    send(new_socket, hello.c_str(), strlen(hello.c_str()), 0); 
    silentRunning=true;
    sendRobotState();

    fcntl(new_socket, F_SETFL, O_NONBLOCK);

    std::list<uint8_t> messageBytesList;
    uint8_t message[256];
    while(rclcpp::ok()){
        bytesRead = recv(new_socket, buffer, 1024, 0);
        for(int index=0;index<bytesRead;index++){
            messageBytesList.push_back(buffer[index]);
        }

        if(bytesRead==0){
            stopPublisher->publish(empty);
	    RCLCPP_INFO(nodeHandle->get_logger(),"Lost Connection");
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
            bytesRead = read(new_socket, buffer, 1024); 
            send(new_socket, hello.c_str(), strlen(hello.c_str()), 0); 
            fcntl(new_socket, F_SETFL, O_NONBLOCK);
    
            silentRunning=true;
            sendRobotState();
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
            //parse command
            uint8_t command=message[0];
            if(command==1){
                messages::msg::AxisState axisState;
                axisState.joystick=message[1];
                if(message[2]==0)
                    state0=parseFloat(&message[3]);
                if(message[2]==1)
                    state1=parseFloat(&message[3]);
                if(message[2]==2)
                    state2=parseFloat(&message[3]);
                axisState.state0=state0;
                axisState.state1=state1;
                axisState.state2=state2;
                joystickAxisPublisher->publish(axisState);
                RCLCPP_INFO(nodeHandle->get_logger(), "joystick %d %f %f %f", axisState.joystick, axisState.state0, axisState.state1, axisState.state2);
            }

            if(command==2){
				// command == 2 keyPublisher
                messages::msg::KeyState keyState;
                keyState.key=((uint16_t)message[1])<<8 | ((uint16_t)message[2]);
                keyState.state=message[3];
                keyPublisher->publish(keyState);
		RCLCPP_INFO(nodeHandle->get_logger(),"key %d %d ", keyState.key , keyState.state);
            }

            if(command==5){
				// command == 5 joystickButtonPublisher
                messages::msg::ButtonState buttonState;
                buttonState.joystick=message[1];
                buttonState.button=message[2];
                buttonState.state=message[3];
                if(buttonState.button==0 && buttonState.state==0){
                    std::cout << "publish stop" << std::endl;
                    stopPublisher->publish(empty);
                }    
                if(buttonState.button==0 && buttonState.state==1){
                    std::cout << "publish go" << std::endl;
                    goPublisher->publish(empty);
                }    
                joystickButtonPublisher->publish(buttonState);
		RCLCPP_INFO(nodeHandle->get_logger(),"button %d %d %d", buttonState.joystick , buttonState.button , buttonState.state);
            }
            if(command==6){
				// command ==6 joystickHatPublisher
                messages::msg::HatState hatState;
                hatState.joystick=message[1];
                hatState.hat=message[2];
                hatState.state=message[3];
                joystickHatPublisher->publish(hatState);
		RCLCPP_INFO(nodeHandle->get_logger(),"hat %d %d %d", hatState.joystick , hatState.hat , hatState.state);
            }
            if(command==7){
				// command == 7 silentRunning
                silentRunning=message[1];
                std::cout << "silentRunning " << silentRunning << std::endl;
            }
            if(command==8){
				// command == 8 reboot
                reboot();
                std::cout << "reboot " << silentRunning << std::endl;
            }
        }
        rclcpp::spin_some(nodeHandle);
    }
	// let the next thread begin execution
    broadcastThread.join();
}
