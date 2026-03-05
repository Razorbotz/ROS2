#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <string>
#include <list>

#include <linux/can.h>
#include <linux/can/raw.h>


class CANSparkMax{
    const char* can_bus_name;
    int can_id;
 //   CANSparkMax::MotorType motor_type;

    int s;
    int nbytes;
    struct sockaddr_can addr;
    struct can_frame frame;
    struct ifreq ifr;

    int applied_output;
    int faults;
    int sticky_faults;
    int invert_brake_follower;  // should be three different fields

    int motor_velocity;
    int motor_temperature;
    int motor_voltage;
    int motor_current;

    int motor_position;
    
    int adc_voltage;
    int analog_velocity;
    float analog_position;

    float alt_encoder_velocity;
    float alt_encoder_position;


    public:
    
    CANSparkMax(std::string can_bus_name, int can_id /*,CANSparkMax::MotorType motor_type*/);

    int get_applied_output();
    int get_faults();
    int get_sticky_faults();
    int get_invert_brake_follower();
    int get_motor_velocity();
    int get_motor_temperature();
    int get_motor_voltage();
    int get_motor_current();
    int get_motor_position();
    int get_adc_voltage();
    int get_analog_velocity();
    float get_analog_position();
    float get_alt_encoder_velocity();
    float get_alt_encoder_position();


    void send_frame(struct can_frame frame);

    void send_heartbeat();
    void broadcast_enumerate();
    void set_setpoint(float setpoint, int pid_controller);
    void set_duty_cycle(float duty_cycle, int pid_controller);
    void set_speed(float speed, int pid_controller);
    void set_position(float position, int pid_controller);
    void set_voltage(float voltage, int pid_controller);
    void set_periodic_status_0(int time_miliseconds);
    void set_periodic_status_1(int time_miliseconds);
    void set_periodic_status_2(int time_miliseconds);
    void set_periodic_status_3(int time_miliseconds);
    void set_follower_mode(uint follower_ID, uint follower_configuration);
    void clear_faults();
    void receive();
    void nack();
    void ack();
    void sync();
    void parameterAccess(int parameterID, float parameterValue);
    void telemetry_update_position(int position);        // not sure about he parameter
    void telemetry_update_accumulator(int accumulator);  // not sure about the parameter

    void loop();

    struct can_frame get_frame(int clas, int index);
    void process_perodic(struct can_frame& frame);
    void process_periodic_status_0(struct can_frame& frame);
    void process_periodic_status_1(struct can_frame& frame);
    void process_periodic_status_2(struct can_frame& frame);
    void process_periodic_status_3(struct can_frame& frame);

    struct Parameter{
        uint8_t id;
	uint8_t data[4];
	uint8_t type;
	bool response;

	Parameter(){
	}
	Parameter(uint8_t id, int value){
            this->id = id;
	    set_int(value);
	}
	Parameter(uint8_t id, uint value){
            this->id = id;
	    set_uint(value);
	}
	Parameter(uint8_t id, float value){
            this->id = id;
	    set_float(value);
	}
	Parameter(uint8_t id, bool value){
            this->id = id;
	    set_bool(value);
	}

        void set_int(int value){
	    data[0] = value & 0xff;
	    data[1] = (value >> 8 ) & 0xff;
	    data[2] = (value >> 16) & 0xff;
	    data[3] = (value >> 24) & 0xff;
            
	    type = 0; 
	}
        int get_int(){
            int value=0;
	    value |= int(data[0]);
	    value |= int(data[1]) << 8;
	    value |= int(data[2]) << 16;
	    value |= int(data[3]) << 24;
           
	    return value;
	}
        void set_uint(uint value){
	    data[0] = value & 0xff;
	    data[1] = (value >> 8 ) & 0xff;
	    data[2] = (value >> 16) & 0xff;
	    data[3] = (value >> 24) & 0xff;
            
	    type = 1; 
	}
        uint get_uint(){	
            uint value=0;
	    value |= uint(data[0]);
	    value |= uint(data[1]) << 8;
	    value |= uint(data[2]) << 16;
	    value |= uint(data[3]) << 24;
           
	    return value;
	}
	void set_float(float value){
	    uint* value_pointer = (uint*)&value;
	    data[0] = (*value_pointer) & 0xff;
	    data[1] = (*value_pointer >> 8 ) & 0xff;
	    data[2] = (*value_pointer >> 16) & 0xff;
	    data[3] = (*value_pointer >> 24) & 0xff;
             
            type = 2;
	}
        float get_float(){
            float value=0;
	    uint* value_pointer = (uint*)&value;
	    *value_pointer = uint(data[0]);
	    *value_pointer |= uint(data[1]) << 8; 
	    *value_pointer |= uint(data[2]) << 16; 
	    *value_pointer |= uint(data[3]) << 24; 

	    return value;
	}
	void set_bool(bool value){
	    data[0] = value & 0xff;

	    type = 3;
	}
	bool get_bool(){
            return (bool)data[0];
	}

    };
    CANSparkMax::Parameter get_config_parameter(int parameter_id);
    CANSparkMax::Parameter set_config_parameter(CANSparkMax::Parameter parameter);

    struct DRVStatus{
	int stat_0;
	int stat_1;
	int faults;
	int sticky_faults;
    };
    CANSparkMax::DRVStatus get_DRV_status();

};

