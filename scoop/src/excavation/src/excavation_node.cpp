#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int16MultiArray.hpp>

rclcpp::Node::SharedPtr nodeHandle;

std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > talon14Publisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > talon15Publisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > talon16Publisher;


/*
enum Error {
    ActuatorsSyncError,
    ActuatorNotMovingError,
    PotentiometerError,
    None
};

std::map<MyEnum, const char*> map;
map_init(map)
    (ActuatorsSyncError, "ActuatorsSyncError")
    (ActuatorNotMovingError, "ActuatorNotMovingError")
    (PotentiometerError, "PotentiometerError")
    (None, "None")
;

struct LinearActuator{
    float speed = 0.0;
    int potentiometer = 0;
    int timeWithoutChange = 0;
    int max = 0;
    int min = 1024;
    Error error = PotentiometerError;
    bool run = true;
    bool atMin = false;
    bool atMax = false;
    float stroke = 11.8;
};

LinearActuator linear1;
LinearActuator linear2;
LinearActuator linear3;
*/

float speed1 = 0;
float speed2 = 0;
int potentiometer1 = 0;
int potentiometer2 = 0;
int count1 = 0;
int count2 = 0;
int max1 = 0;
int max2 = 0;
int min1 = 1024;
int min2 = 1024;
int thresh1 = 60;
int thresh2 = 120;
int thresh3 = 180;

bool automationGo = false;


void sync(){
    if(speed->data > 0){
        float diff = abs(potentiometer1 - potentiometer2);
        if (diff > thresh3){
            (potentiometer1 > potentiometer2) ? speed1 = 0 : speed2 = 0;
            // Raise error
        }
        else if (diff > thresh2){
            (potentiometer1 > potentiometer2) ? speed1 *= 0.5 : speed2 *= 0.5;
        }
        else if (diff > thresh1){
            (potentiometer1 > potentiometer2) ? speed1 *= 0.9 : speed2 *= 0.9;
        }
    }
    else{
        float diff = abs(potentiometer1 - potentiometer2);
        if (diff > thresh3){
            (potentiometer1 < potentiometer2) ? speed1 = 0 : speed2 = 0;
            // Raise error
        }
        else if (diff > thresh2){
            (potentiometer1 < potentiometer2) ? speed1 *= 0.5 : speed2 *= 0.5;
        }
        else if (diff > thresh1){
            (potentiometer1 < potentiometer2) ? speed1 *= 0.9 : speed2 *= 0.9;
        }
    }
}


void shoulderCallback(const std_msgs::msg::Float32::SharedPtr speed){
    speed1 = speed->data;
    speed2 = speed->data;
    //sync();
    talon14Publisher->publish(speed1);
    talon15Publisher->publish(speed2);
}


void potentiometer1Callback(const std_msgs::msg::Int16MultiArray::SharedPtr potent){
    RCLCPP_INFO(nodeHandle->get_logger(),"Potentiometer %d %d", potent->data[0], potent->data[1]);
    /*
    if(potent->data[0] == 1024){
        // Raise error
    }
    else if(potent->data[1] == 1024){
        // Raise error
    }
    else{
        if(potentiometer1 == potent->data[0] && speed1 != 0){
            count1 += 1;
        }
        else{
            count1 = 0;
        }
        if(potent->data[0] < min1){
            min1 = potent->data[0];
        }
        if(potent->data[0] > max1){
            max1 = potent->data[0];
        }
        if(count1 == 5 && potent->data[0] == min1 && speed1 < 0){
            speed1 = 0;
        }
        if(count1 == 5 && potent->data[0] == max1 && speed1 > 0){
            speed1 = 0;
        }
        if(count1 == 5 && potent->data[0] != max1 && potent->data[0] != min1 && speed1 != 0){
            //Raise error
            speed1 = 0;
            speed2 = 0;
        }


        if(potentiometer2 == potent->data[1] && speed2 != 0){
            count2 += 1;
        }
        else{
            count2 = 0;
        }
        if(potent->data[1] < min2){
            min2 = potent->data[1];
        }
        if(potent->data[1] > max2){
            max2 = potent->data[1];
        }
        if(count2 == 5 && potent->data[1] == min2 && speed2 < 0){
            speed2 = 0;
        }
        if(count2 == 5 && potent->data[1] != max2 && potent->data[1] != min2 && speed2 != 0){
            //Raise error
            speed1 = 0;
            speed2 = 0;
        }
        
        potentiometer1 = potent->data[0];
        potentiometer2 = potent->data[1];
        sync();
    }
    */
}


void automationGoCallback(const std_msgs::msg::Bool::SharedPtr msg){
    automationGo = msg->data;
}


int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    nodeHandle = rclcpp::Node::make_shared("excavation");

    auto potentiometerSubscriber = nodeHandle->create_subscription<std_msgs::msg::Int16MultiArray>("potentiometer_data",1, potentiometerCallback);
    auto shoulderSubscriber = nodeHandle->create_subscription<std_msgs::msg::Float32>("shoulder_speed",1,shoulderCallback);
    auto automationGoSubscriber = nodeHandle->create_subscription<std_msgs::msg::Bool>("automationGo",1,automationGoCallback);

    talon14Publisher= nodeHandle->create_publisher<std_msgs::msg::Float32>("talon_14_speed",1);
    talon15Publisher= nodeHandle->create_publisher<std_msgs::msg::Float32>("talon_15_speed",1);
    talon16Publisher= nodeHandle->create_publisher<std_msgs::msg::Float32>("talon_16_speed",1);
    
    rclcpp::Rate rate(20);
    while(rclcpp::ok()){
        rclcpp:spin_some(nodeHandle);
        rate.sleep();
    }
}