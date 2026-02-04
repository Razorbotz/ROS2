#include "utils/utils.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>

namespace utils{
	void set_speed_frame(struct can_frame* frame, int motorNumber, float speed){
        frame->can_id  = 0x02040080 | motorNumber;
        frame->can_id |= CAN_EFF_FLAG;
        frame->can_dlc = 8;
        if(motorNumber == 14){
            frame->data[3] = 0x00;
            frame->data[4] = 0x00;
            frame->data[5] = 0x0B;
            frame->data[7] = 0xC0;
            if(speed < -0.8){
                frame->data[0] = 0xC8;
                frame->data[1] = 0x28;
                frame->data[2] = 0x03;
                frame->data[6] = 0x10;
                //C8 28 03 00 00 0B 10 C0
            }
            else if(speed < -0.6){
                frame->data[0] = 0xD1;
                frame->data[1] = 0xA1;
                frame->data[2] = 0x03;
                frame->data[6] = 0x10;
                //D1 A1 03 00 00 0B 10 C0
            }
            else if(speed < -0.4){
                frame->data[0] = 0xF8;
                frame->data[1] = 0x68;
                frame->data[2] = 0x03;
                frame->data[6] = 0x10;
                //F8 68 03 00 00 0B 10 C0
            }
            else if(speed < -0.2){
                frame->data[0] = 0x85;
                frame->data[1] = 0xA8;
                frame->data[2] = 0x03;
                frame->data[6] = 0x00;
                //85 A8 03 00 00 0B 10 C0 
            }
            else if (speed < 0.2){
                frame->data[0] = 0x44;
                frame->data[1] = 0xF8;
                frame->data[2] = 0x06;
                frame->data[6] = 0x00;
                //44 F8 06 00 00 0B 00 C0
            }
            else if(speed < 0.4){
                frame->data[0] = 0x79;
                frame->data[1] = 0xF0;
                frame->data[2] = 0x02;
                frame->data[6] = 0x10;
                //79 F0 02 00 00 0B 10 C0 
            }
            else if(speed < 0.6){
                frame->data[0] = 0x07;
                frame->data[1] = 0xF0;
                frame->data[2] = 0x02;
                frame->data[6] = 0x10;
                //07 F0 02 00 00 0B 10 C0
            }
            else if(speed < 0.8){
                frame->data[0] = 0x2B;
                frame->data[1] = 0x30;
                frame->data[2] = 0x02;
                frame->data[6] = 0x10;
                //2B 30 02 00 00 0B 10 C0
            }
            else{
                frame->data[0] = 0x33;
                frame->data[1] = 0xF0;
                frame->data[2] = 0x02;
                frame->data[6] = 0x10;
                //33 F0 02 00 00 0B 10 C0
            }
        }
        if(motorNumber == 16){
            frame->data[3] = 0x00;
            frame->data[4] = 0x00;
            frame->data[5] = 0x0B;
            frame->data[7] = 0xC0;
            if(speed < -0.8){
                frame->data[0] = 0x57;
                frame->data[1] = 0x58;
                frame->data[2] = 0x06;
                frame->data[6] = 0x10;
                //57 58 06 00 00 0B 10 C0
            }
            else if(speed < -0.6){
                frame->data[0] = 0x4D;
                frame->data[1] = 0x78;
                frame->data[2] = 0x06;
                frame->data[6] = 0x00;
                //4D 78 06 00 00 0B 00 C0 
            }
            else if(speed < -0.4){
                frame->data[0] = 0x64;
                frame->data[1] = 0x58;
                frame->data[2] = 0x06;
                frame->data[6] = 0x10;
                //64 58 06 00 00 0B 10 C0
            }
            else if(speed < -0.2){
                frame->data[0] = 0x0C;
                frame->data[1] = 0x78;
                frame->data[2] = 0x06;
                frame->data[6] = 0x00;
                //0C 78 06 00 00 0B 00 C0
            }
            else if (speed < 0.2){
                frame->data[0] = 0x31;
                frame->data[1] = 0x90;
                frame->data[2] = 0x02;
                frame->data[6] = 0x10;
                //31 90 02 00 00 0B 10 C0
            }
            else if(speed < 0.4){
                frame->data[0] = 0xF1;
                frame->data[1] = 0x78;
                frame->data[2] = 0x06;
                frame->data[6] = 0x10;
                //F1 78 06 00 00 0B 10 C0 
            }
            else if(speed < 0.6){
                frame->data[0] = 0x9E;
                frame->data[1] = 0x58;
                frame->data[2] = 0x06;
                frame->data[6] = 0x00;
                //9E 58 06 00 00 0B 00 C0
            }
            else if(speed < 0.8){
                frame->data[0] = 0xB6;
                frame->data[1] = 0x38;
                frame->data[2] = 0x06;
                frame->data[6] = 0x00;
                //B6 38 06 00 00 0B 00 C0
            }
            else{
                frame->data[0] = 0xA8;
                frame->data[1] = 0x98;
                frame->data[2] = 0x06;
                frame->data[6] = 0x00;
                //A8 98 06 00 00 0B 00 C0 
            }
        }
    }
}
