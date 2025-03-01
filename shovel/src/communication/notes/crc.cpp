#include <iostream>
#include <vector>
#include <list>
#include <memory>
#include <iomanip>

int key = 0x2C;
void crc_encode(std::shared_ptr<std::list<uint8_t>> byteList) {
    //Get length of byteList 

    //Append key-1 zeros at end of bytelist

    //Perform mod2division (appended_data,key)

    //

}


void checksum_encode(std::shared_ptr<std::list<uint8_t>> byteList){
    uint32_t sum = 0;  // Use a wider type to avoid overflow

    // Append zero byte as placeholders for the checksum
    byteList->push_back(0x00);


    std::cout << "Bytes with placeholders: ";
    for (auto byte : *byteList) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    }
    std::cout << std::endl;

    // Sum all the bytes
    for (uint8_t byte : *byteList) {
        sum += byte;
    }

    // Compute Checksum
    uint8_t checksum = sum % key;
    std::cout << "Simple checksum computed: 0x" << std::hex << static_cast<int>(checksum) << std::endl;

    
    auto it = byteList->end();
    std::advance(it, -1);
    *it = checksum;

    std::cout << "Final byteList: ";
    for (auto byte : *byteList) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    }
    std::cout << std::endl;
}

void checksum_decode(std::shared_ptr<std::list<uint8_t>> byteList){
    //Checks last two bytes for the checksum
    if (byteList->size() < 1) {
        std::cout << "Not enough data to decode checksum." << std::endl;
        return;
    }

    // Extracts checksum
    auto it = byteList->end();
    std::advance(it, -1);
    uint8_t storedChecksum = *it;

    // Sums byteList, excludes last two bytes 
    uint32_t sum = 0;
    auto dataEnd = byteList->end();
    std::advance(dataEnd, -1);
    for (auto dataIt = byteList->begin(); dataIt != dataEnd; ++dataIt) {
        sum += *dataIt;
    }

    // Recalculate the checksum as sum modulo key.
    uint8_t computedChecksum = sum % key;

    std::cout << "Computed checksum from data: 0x" << std::hex << static_cast<int>(computedChecksum) << std::endl;
    std::cout << "Stored checksum: 0x" << std::hex << static_cast<int>(storedChecksum) << std::endl;

    if (computedChecksum == storedChecksum) {
        std::cout << "Checksum is valid." << std::endl;
    } else {
        std::cout << "Checksum is invalid." << std::endl;
    }

}


int main(){

    std::shared_ptr<std::list<uint8_t>> byteList = std::make_shared<std::list<uint8_t>>();

    byteList->push_back(0xAB);
    byteList->push_back(0xCD);
    byteList->push_back(0xEF);


    std::cout << "byteList contents: ";
    for (const auto& byte : *byteList) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    }
    std::cout << std::endl;

    //checksum_encode(byteList);
    crc_encode_crc16(byteList);
    //auto it = byteList->end();
    // std::advance(it, -3);
    // *it = 0x0e;  
    //checksum_decode(byteList);

    
}