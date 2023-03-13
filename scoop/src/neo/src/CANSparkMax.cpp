#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <iostream>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include "CANSparkMax.hpp"

#include <bitset>

CANSparkMax::CANSparkMax(std::string can_bus_name, int can_id /*,CANSparkMax::MotorType motor_type*/){
    this->can_bus_name = can_bus_name.c_str();    
    this->can_id = can_id & 0x1f;
    std::cout << std::hex << this->can_id << std::endl;
//    this->motor_type = motor_type;

    if((this->s = socket(PF_CAN, SOCK_RAW|SOCK_NONBLOCK, CAN_RAW)) < 0) {
        perror("Error while opening socket");
    }


    strcpy(this->ifr.ifr_name, this->can_bus_name);
    ioctl(this->s, SIOCGIFINDEX, &ifr);

    this->addr.can_family  = AF_CAN;
    this->addr.can_ifindex = this->ifr.ifr_ifindex;

    printf("%s at index %d\n", this->can_bus_name, this->ifr.ifr_ifindex);

    if(bind(this->s, (struct sockaddr *)&this->addr, sizeof(this->addr)) < 0) {
        perror("Error in socket bind");
    }
}


int CANSparkMax::get_applied_output(){
    return this->applied_output;
}


int CANSparkMax::get_faults(){
    return this->faults;
}


int CANSparkMax::get_sticky_faults(){
    return this->sticky_faults;
}


int CANSparkMax::get_invert_brake_follower(){
    return this->invert_brake_follower;
}


int CANSparkMax::get_motor_velocity(){
    return this->motor_velocity;
}


int CANSparkMax::get_motor_temperature(){
    return this->motor_temperature;
}


int CANSparkMax::get_motor_voltage(){
    return this->motor_voltage;
}


int CANSparkMax::get_motor_current(){
    return this->motor_current;
}


int CANSparkMax::get_motor_position(){
    return this->motor_position;
}


int CANSparkMax::get_adc_voltage(){
    return this->adc_voltage;
}


int CANSparkMax::get_analog_velocity(){
    return this->analog_velocity;
}


float CANSparkMax::get_analog_position(){
    return this->analog_position;
}


float CANSparkMax::get_alt_encoder_velocity(){
    return this->alt_encoder_velocity;
}


float CANSparkMax::get_alt_encoder_position(){
    return this->alt_encoder_position;
}





void CANSparkMax::send_frame(struct can_frame frame) {
//    std::cout << std::hex << frame.can_id << std::dec << " " << (int)frame.can_dlc << " " << sizeof(struct can_frame) << std::endl;
    nbytes = write(this->s, &frame, sizeof(struct can_frame));
    //nbytes = write(this->s, &frame, 8+frame.can_dlc);
    //nbytes = write(this->s, &frame, 16);
//    printf("Wrote %d bytes\n", nbytes);
}


void CANSparkMax::broadcast_enumerate(){
    struct can_frame frame;
    frame.can_id  = 0x00000240 | CAN_EFF_FLAG;
    frame.can_dlc = 0;

    this->send_frame(frame);
}


void CANSparkMax::send_heartbeat(){
    struct can_frame frame;
    frame.can_id  = 0x02052C80 | CAN_EFF_FLAG ;
    frame.can_dlc = 8;
    frame.data[0] = 0xff;
    frame.data[1] = 0xff;
    frame.data[2] = 0xff;
    frame.data[3] = 0xff;
    frame.data[4] = 0xff;
    frame.data[5] = 0xff;
    frame.data[6] = 0xff;
    frame.data[7] = 0xff;

    this->send_frame(frame);

    this->receive();
}


void CANSparkMax::set_setpoint(float setpoint, int pid_controller){
    struct can_frame frame;
    frame.can_id  = 0x02050040 | CAN_EFF_FLAG;
    frame.can_id |= this->can_id;
    frame.can_dlc = 8;
    int* int_setpoint = (int*) &setpoint;
    frame.data[0] = *int_setpoint & 0xff;
    frame.data[1] = (*int_setpoint >> 8 ) & 0xff;
    frame.data[2] = (*int_setpoint >> 16) & 0xff;
    frame.data[3] = (*int_setpoint >> 24) & 0xff;
    frame.data[4] = 0;
    frame.data[5] = 0;
    frame.data[6] = pid_controller & 0x3;
    frame.data[7] = 0;

    this->send_frame(frame);
}


void CANSparkMax::set_duty_cycle(float duty_cycle, int pid_controller){
    struct can_frame frame;
    frame.can_id  = 0x02050080 | CAN_EFF_FLAG;
    frame.can_id |= this->can_id;
    frame.can_dlc = 8;
    int* int_duty_cycle = (int*) &duty_cycle;
    frame.data[0] = *int_duty_cycle & 0xff;
    frame.data[1] = (*int_duty_cycle >> 8 ) & 0xff;
    frame.data[2] = (*int_duty_cycle >> 16) & 0xff;
    frame.data[3] = (*int_duty_cycle >> 24) & 0xff;
    frame.data[4] = 0;
    frame.data[5] = 0;
    frame.data[6] = pid_controller & 0x3;
    frame.data[7] = 0;

    this->send_frame(frame);
}


void CANSparkMax::set_speed(float speed, int pid_controller){
    struct can_frame frame;
    frame.can_id  = 0x02050480 | CAN_EFF_FLAG;
    frame.can_id |= this->can_id;
    frame.can_dlc = 8;
    int* int_speed = (int*) &speed;
    frame.data[0] = *int_speed & 0xff;
    frame.data[1] = (*int_speed >> 8 ) & 0xff;
    frame.data[2] = (*int_speed >> 16) & 0xff;
    frame.data[3] = (*int_speed >> 24) & 0xff;
    frame.data[4] = 0;
    frame.data[5] = 0;
    frame.data[6] = pid_controller & 0x3;
    frame.data[7] = 0;


    this->send_frame(frame);
}


void CANSparkMax::set_position(float position, int pid_controller){
    struct can_frame frame;
    frame.can_id  = 0x02050C80 | CAN_EFF_FLAG;
    frame.can_id |= this->can_id;
    frame.can_dlc = 8;
    int* int_position = (int*) &position;
    frame.data[0] = *int_position & 0xff;
    frame.data[1] = (*int_position >> 8 ) & 0xff;
    frame.data[2] = (*int_position >> 16) & 0xff;
    frame.data[3] = (*int_position >> 24) & 0xff;
    frame.data[4] = 0;
    frame.data[5] = 0;
    frame.data[6] = pid_controller & 0x3;
    frame.data[7] = 0;

    this->send_frame(frame);
}


void CANSparkMax::set_voltage(float voltage, int pid_controller){
    struct can_frame frame;
    frame.can_id  = 0x02051080 | CAN_EFF_FLAG;
    frame.can_id |= this->can_id;
    frame.can_dlc = 8;
    int* int_voltage = (int*) &voltage;
    frame.data[0] = *int_voltage & 0xff;
    frame.data[1] = (*int_voltage >> 8 ) & 0xff;
    frame.data[2] = (*int_voltage >> 16) & 0xff;
    frame.data[3] = (*int_voltage >> 24) & 0xff;
    frame.data[4] = 0;
    frame.data[5] = 0;
    frame.data[6] = pid_controller & 0x3;
    frame.data[7] = 0;

    this->send_frame(frame);
}


struct can_frame CANSparkMax::get_frame(int clas, int index){
    struct can_frame frame;

    while(true){
        int nbytes = read(this->s, &frame, sizeof(struct can_frame));
        if(nbytes > 0){
            int device_type  = (frame.can_id >> 24) & 0x1f;
            int manufacturer = (frame.can_id >> 16) & 0xff;
            int api_class    = (frame.can_id >> 10) & 0x3f;
            int api_index    = (frame.can_id >>  6) & 0x0f;
            int device_id    = (frame.can_id      ) & 0x3f;
	    if (device_type == 2 && manufacturer == 5 && device_id == this->can_id){ 
                if (api_class==clas && api_index==index) {
		    break;
		}else{
	            process_perodic(frame);	
		}
	    }
	}
    }

    return frame;
}


CANSparkMax::Parameter CANSparkMax::get_config_parameter(int parameter_id){
    CANSparkMax::Parameter parameter;
    struct can_frame frame;
    frame.can_id  = 0x02051C40 | CAN_EFF_FLAG;
    frame.can_id |= this->can_id;
    frame.can_dlc = 2;
    frame.data[0] = parameter_id & 0xff;
    frame.data[1] = 0;

    this->send_frame(frame);

    frame = this->get_frame(7,1);

    parameter.id = frame.data[0];
    parameter.data[0] = frame.data[2];
    parameter.data[1] = frame.data[3];
    parameter.data[2] = frame.data[4];
    parameter.data[3] = frame.data[5];
    parameter.type = frame.data[6];
    parameter.response = frame.data[7];

//    while(true){
//        int nbytes = read(s, &frame, sizeof(struct can_frame));
//        if(nbytes > 0){
//            int device_type  = (frame.can_id >> 24) & 0x1f;
//            int manufacturer = (frame.can_id >> 16) & 0xff;
//            int api_class    = (frame.can_id >> 10) & 0x3f;
//            int api_index    = (frame.can_id >>  6) & 0x0f;
//            int device_id    = (frame.can_id      ) & 0x3f;
//	    if (device_type == 2 && manufacturer == 5 && device_id == this->can_id){ 
//                if (api_class==7 && api_index==1) {
//                  parameter.id = frame.data[0];
//		    parameter.data[0] = frame.data[2];
//		    parameter.data[1] = frame.data[3];
//		    parameter.data[2] = frame.data[4];
//		    parameter.data[3] = frame.data[5];
//		    parameter.type = frame.data[6];
//		    parameter.response = frame.data[7];
//		    break;
//		}else{
//	            process_perodic(frame);	
//		}
//	    }
//	}
//    }
    return parameter;
}


CANSparkMax::Parameter CANSparkMax::set_config_parameter(CANSparkMax::Parameter parameter){
    struct can_frame frame;
    frame.can_id = 0x02051C00 | CAN_EFF_FLAG;
    frame.can_id |= this->can_id;
    frame.can_dlc = 7;
    frame.data[0] = parameter.id;   
    frame.data[1] = 0xff; 
    frame.data[2] = parameter.data[0]; 
    frame.data[3] = parameter.data[1]; 
    frame.data[4] = parameter.data[2]; 
    frame.data[5] = parameter.data[3]; 
    frame.data[6] = parameter.type;

    this->send_frame(frame);

    frame = get_frame(7,0);


    CANSparkMax::Parameter parameter_out;

    parameter_out.id = frame.data[0];
    parameter_out.data[0] = frame.data[2];
    parameter_out.data[1] = frame.data[3];
    parameter_out.data[2] = frame.data[4];
    parameter_out.data[3] = frame.data[5];
    parameter_out.type = frame.data[6];
    parameter_out.response = frame.data[7];

    return parameter_out;
}


CANSparkMax::DRVStatus CANSparkMax::get_DRV_status(){
    CANSparkMax::DRVStatus drv_status;
    struct can_frame frame;
    frame.can_id  = 0x02051A80 | CAN_EFF_FLAG | this->can_id;
    frame.can_dlc = 0;

    this->send_frame(frame);

    frame = get_frame(7, 10);

    drv_status.stat_0 = frame.data[0];  	
    drv_status.stat_0 |= frame.data[1] << 8; 	
    drv_status.stat_1 = frame.data[2]; 	
    drv_status.stat_1 |= frame.data[3] << 8;	
    drv_status.faults = frame.data[4];  	
    drv_status.faults |= frame.data[5] << 8; 
    drv_status.sticky_faults = frame.data[6];
    drv_status.sticky_faults |= frame.data[7] << 8; 

//    while(true){
//        int nbytes = read(s, &frame, sizeof(struct can_frame));
//        if(nbytes > 0){
//            int device_type  = (frame.can_id >> 24) & 0x1f;
//            int manufacturer = (frame.can_id >> 16) & 0xff;
//            int api_class    = (frame.can_id >> 10) & 0x3f;
//            int api_index    = (frame.can_id >>  6) & 0x0f;
//            int device_id    = (frame.can_id      ) & 0x3f;
//	    if (device_type == 2 && manufacturer == 5 && device_id == this->can_id){ 
//                if (api_class==7 && api_index==10) {
//		    drv_status.stat_0 = frame.data[0];  	
//		    drv_status.stat_0 |= frame.data[1] << 8; 	
//		    drv_status.stat_1 = frame.data[2]; 	
//		    drv_status.stat_1 |= frame.data[3] << 8;	
//		    drv_status.faults = frame.data[4];  	
//		    drv_status.faults |= frame.data[5] << 8; 
//	            drv_status.sticky_faults = frame.data[6];
//	            drv_status.sticky_faults |= frame.data[7] << 8; 
//		    break;
//		}else{
//	            process_perodic(frame);	
//		}
//	    }
//	}
//    }

    return drv_status;
}


void CANSparkMax::set_follower_mode(uint follower_ID, uint follower_configuration){
    struct can_frame frame;
    frame.can_id = 0x02051CC0 | CAN_EFF_FLAG | this->can_id;

    frame.can_dlc = 8;
    frame.data[0] = follower_ID & 0xff;   
    frame.data[1] = (follower_ID >> 8 ) & 0xff;   
    frame.data[2] = (follower_ID >> 16 ) & 0xff;   
    frame.data[3] = (follower_ID >> 24 ) & 0xff;   
    frame.data[4] = follower_configuration & 0xff;   
    frame.data[5] = (follower_configuration >> 8 ) & 0xff;   
    frame.data[6] = (follower_configuration >> 16 ) & 0xff;   
    frame.data[7] = (follower_configuration >> 24 ) & 0xff;   

    this->send_frame(frame);
    
    frame = get_frame(7, 3);

    // I don't see the point in decoding the frame for now so I'm leaving it undone

}


void CANSparkMax::telemetry_update_position(int position){
    struct can_frame frame;
    frame.can_id = 0x02052800 | CAN_EFF_FLAG | this->can_id;

    frame.can_dlc = 4;
    frame.data[0] = position & 0xff;   
    frame.data[1] = (position >> 8 ) & 0xff;   
    frame.data[2] = (position >> 16 ) & 0xff;   
    frame.data[3] = (position >> 24 ) & 0xff;   

    this->send_frame(frame);
}


void CANSparkMax::telemetry_update_accumulator(int accumulator){
    struct can_frame frame;
    frame.can_id = 0x02052880 | CAN_EFF_FLAG | this->can_id;

    frame.can_dlc = 4;
    frame.data[0] = accumulator & 0xff;   
    frame.data[1] = (accumulator >> 8 ) & 0xff;   
    frame.data[2] = (accumulator >> 16 ) & 0xff;   
    frame.data[3] = (accumulator >> 24 ) & 0xff;   

    this->send_frame(frame);
}


void CANSparkMax::nack(){
    struct can_frame frame;
    frame.can_id = 0x02052000 | CAN_EFF_FLAG | this->can_id;

    frame.can_dlc = 0;
    this->send_frame(frame);
}


void CANSparkMax::ack(){
    struct can_frame frame;
    frame.can_id = 0x02052040 | CAN_EFF_FLAG | this->can_id;

    frame.can_dlc = 0;
    this->send_frame(frame);
}


void CANSparkMax::sync(){
    struct can_frame frame;
    frame.can_id = 0x020524C0 | CAN_EFF_FLAG | this->can_id;

    frame.can_dlc = 0;
    this->send_frame(frame);
}


void CANSparkMax::parameterAccess(int parameterID, float parameterValue){
    struct can_frame frame;
    frame.can_id = 0x020C000 | CAN_EFF_FLAG | parameterID;

    frame.can_dlc = 8;
    frame.data[0] = 0;
    frame.data[1] = 0;
    frame.data[2] = 0;
    frame.data[3] = 0;
    frame.data[4] = 0;
    frame.data[5] = 0;
    frame.data[6] = 0;
    frame.data[7] = 0;

    this->send_frame(frame);
}


void CANSparkMax::set_periodic_status_0(int time_miliseconds){
    struct can_frame frame;
    frame.can_id  = 0x2051800 | CAN_EFF_FLAG;
    frame.can_id |= this->can_id;
    frame.can_dlc = 2;
    frame.data[0] = time_miliseconds & 0xff;
    frame.data[1] = (time_miliseconds >> 8 ) & 0xff;

    this->send_frame(frame);    
}


void CANSparkMax::set_periodic_status_1(int time_miliseconds){
    struct can_frame frame;
    frame.can_id  = 0x2051840 | CAN_EFF_FLAG;
    //frame.can_id |= this->can_id;
    frame.can_dlc = 2;
    frame.data[0] = time_miliseconds & 0xff;
    frame.data[1] = (time_miliseconds >> 8 ) & 0xff;

    this->send_frame(frame);    
}


void CANSparkMax::set_periodic_status_2(int time_miliseconds){
    struct can_frame frame;
    frame.can_id  = 0x2051880 | CAN_EFF_FLAG;
    frame.can_id |= this->can_id;
    frame.can_dlc = 2;
    frame.data[0] = time_miliseconds & 0xff;
    frame.data[1] = (time_miliseconds >> 8 ) & 0xff;

    this->send_frame(frame);    
}


void CANSparkMax::set_periodic_status_3(int time_miliseconds){
    struct can_frame frame;
    frame.can_id  = 0x20518C0 | CAN_EFF_FLAG;
    frame.can_id |= this->can_id;
    frame.can_dlc = 2;
    frame.data[0] = time_miliseconds & 0xff;
    frame.data[1] = (time_miliseconds >> 8 ) & 0xff;

    this->send_frame(frame);    
}


void CANSparkMax::clear_faults(){
    struct can_frame frame;
    frame.can_id  = 0x02051B80 | CAN_EFF_FLAG;
    frame.can_id |= this->can_id;
    frame.can_dlc = 0;

    this->send_frame(frame);
}


void CANSparkMax::process_periodic_status_0(struct can_frame& frame){
    //std::cout << "process_periodic_status_0 " << std::hex << frame.can_id << std::endl;
    this->applied_output = int(frame.data[0]);
    this->applied_output |= int(frame.data[1]) << 8;
    //std::cout << "applied output: " << this->applied_output << std::endl;
    this->faults = int(frame.data[2]);
    this->faults |= int(frame.data[3]) << 8;
    //std::cout << "faults: " << this->faults << std::endl;
    this->sticky_faults = int(frame.data[4]);
    this->sticky_faults |= int(frame.data[5]) << 8;
    //std::cout << "sticky faults: " << this->sticky_faults << std::endl;
    this->invert_brake_follower = int(frame.data[7]);
    //std::cout << "invert brake follower: " << this->invert_brake_follower << std::endl;
}


void CANSparkMax::process_periodic_status_1(struct can_frame& frame){
    //std::cout << "process_periodic_status_1 " << std::hex << frame.can_id << std::endl;
    this->motor_velocity  = frame.data[0];
    this->motor_velocity |= frame.data[1] << 8;
    this->motor_velocity |= frame.data[2] << 16;
    this->motor_velocity |= frame.data[3] << 24;
    //std::cout << "motor velocity: " << float(this->motor_velocity) << std::endl;
    this->motor_temperature = int(frame.data[4]);
    //std::cout << "motor temperature: " << this->motor_temperature << std::endl;
    this->motor_voltage = frame.data[5];
    //std::cout << "motor voltage: " << float(this->motor_voltage) << std::endl;
    this->motor_current = int(frame.data[6]) & 0xf0 >> 4;
    this->motor_current |= int(frame.data[7]) << 4;
    //std::cout << "motor current: " << this->motor_current << std::endl;
}


void CANSparkMax::process_periodic_status_2(struct can_frame& frame){
    //std::cout << "process_periodic_status_2 " << std::hex << frame.can_id << std::endl;
    this->motor_position  = int(frame.data[0]);
    this->motor_position |= int(frame.data[1]) << 8;
    this->motor_position |= int(frame.data[2]) << 16;
    this->motor_position |= int(frame.data[3]) << 24;
    //std::cout << "motor position: " << this->motor_position << std::endl;
}


void CANSparkMax::process_periodic_status_3(struct can_frame& frame){
    //std::cout << "process_periodic_status_3 " << std::hex << frame.can_id << std::endl;
    this->adc_voltage  = int(frame.data[0]);
    this->adc_voltage |= int(frame.data[1]) & 0x03 << 8;
    //std::cout << "adc voltage: " << this->adc_voltage << std::endl;
    this->analog_velocity |= int(frame.data[2]) & 0xfc >> 2;
    this->analog_velocity |= int(frame.data[3]) << 6;
    this->analog_velocity |= int(frame.data[4]) << 14;
    //std::cout << "analog velocity: " << this->analog_velocity << std::endl;
    int* analog_position = (int*)&this->analog_position;
    *analog_position  = int(frame.data[4]);
    *analog_position |= int(frame.data[5]) << 8;
    *analog_position |= int(frame.data[6]) << 16;
    *analog_position |= int(frame.data[7]) << 24;
    //std::cout << "analog position: " << this->analog_position << std::endl;
}


void CANSparkMax::process_perodic(struct can_frame& frame){
    int device_type  = (frame.can_id >> 24) & 0x1f;
    int manufacturer = (frame.can_id >> 16) & 0xff;
    int api_class    = (frame.can_id >> 10) & 0x3f;
    int api_index    = (frame.can_id >>  6) & 0x0f;
    int device_id    = (frame.can_id      ) & 0x3f;

    if(device_type == 2 && manufacturer == 5 && device_id == this->can_id){ 
        if (api_class==6 && api_index==0) process_periodic_status_0(frame);
        if (api_class==6 && api_index==1) process_periodic_status_1(frame);
        if (api_class==6 && api_index==2) process_periodic_status_2(frame);
        if (api_class==6 && api_index==3) process_periodic_status_3(frame);
    }
}


void CANSparkMax::receive(){
    struct can_frame frame;
    int nbytes=0;
    while(nbytes != -1){	
        nbytes = read(s, &frame, sizeof(struct can_frame));	
        if(nbytes > 0){
	    process_perodic(frame);	
	}
    } 
}


void CANSparkMax::loop(){
// this needs to run in a separate thread
    while(1){
	this->send_heartbeat();    
//        this->broadcast_enumerate();	
//        this->send_commands();
    this->receive();
	usleep(100);
    }
}
