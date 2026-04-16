#pragma once
#include <string>
#include <list>
#include <vector>
#include <memory>
#include <ifaddrs.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <cstring>
#include <iostream>
#include <unistd.h>
#include <sys/reboot.h>
#include <linux/reboot.h>

/** @brief Parse a byte represenation into a float.
 * 
 * @param array
 * @return value
 * */
inline float parseFloat(uint8_t* array){
    uint32_t axisYInteger=0;
    axisYInteger|=uint32_t(array[0])<<24;    
    axisYInteger|=uint32_t(array[1])<<16;    
    axisYInteger|=uint32_t(array[2])<<8;    
    axisYInteger|=uint32_t(array[3])<<0;    
    return *(static_cast<float*>(static_cast<void*>(&axisYInteger)));
}

int key = 0x2C;
inline void checksum_encode(std::shared_ptr<std::list<uint8_t>> byteList){
    uint32_t sum = 0;  // Use a wider type to avoid overflow

    // Append zero byte as placeholders for the checksum
    byteList->push_back(0x00);


    //std::cout << "Bytes with placeholders: ";
    // for (auto byte : *byteList) {
    //     std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    // }
    //std::cout << std::endl;

    // Sum all the bytes
    for (uint8_t byte : *byteList) {
        sum += byte;
    }

    // Compute Checksum
    uint8_t checksum = sum % key;
    //std::cout << "Simple checksum computed: 0x" << std::hex << static_cast<int>(checksum) << std::endl;

    
    auto it = byteList->end();
    std::advance(it, -1);
    *it = checksum;

    // std::cout << "Final byteList: ";
    // for (auto byte : *byteList) {
    //     std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    // }
    // std::cout << std::endl;
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
inline std::string getAddressString(int family, std::string interfaceName){
    std::string addressString("");
    ifaddrs* ifAddrStruct = nullptr;
    ifaddrs* ifa = nullptr;

    if (getifaddrs(&ifAddrStruct) == 0) {
        
        for (ifa = ifAddrStruct; ifa != nullptr; ifa = ifa->ifa_next) {
            
            if (ifa->ifa_name != nullptr && 
                strcmp(ifa->ifa_name, interfaceName.c_str()) == 0 && 
                ifa->ifa_addr->sa_family == family) {
                
                if (ifa->ifa_addr->sa_family == AF_INET) {
                    sockaddr_in *socketAddress = reinterpret_cast<sockaddr_in *>(ifa->ifa_addr);
                    addressString += inet_ntoa(socketAddress->sin_addr);
                }
                if (ifa->ifa_addr->sa_family == AF_INET6) {
                    sockaddr_in6 *socketAddress = reinterpret_cast<sockaddr_in6 *>(ifa->ifa_addr);
                    for (int index = 0; index < 16; index += 2) {
                        char bits[5];
                        sprintf(bits,"%02x%02x", socketAddress->sin6_addr.s6_addr[index],socketAddress->sin6_addr.s6_addr[index + 1]);
                        if (index) addressString +=":";
                        addressString +=bits;
                    }
                }
                if (ifa->ifa_addr->sa_family == AF_PACKET) {
                    sockaddr_ll *socketAddress = reinterpret_cast<sockaddr_ll *>(ifa->ifa_addr);
                    for (int index = 0; index < socketAddress->sll_halen; index++) {
                        char bits[3];
                        sprintf(bits,"%02x", socketAddress->sll_addr[index]);
                        if (index) addressString +=":";
                        addressString +=bits;
                    }
                }
            }
        }
        freeifaddrs(ifAddrStruct);
    }
    
    return addressString;
}

/** @brief Prints the address
 * 
 * */
inline void printAddresses() {
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

inline void reboot(){
    sync();
    reboot(LINUX_REBOOT_CMD_POWER_OFF);
}