#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>

#include "messages/msg/linear_out.hpp"

rclcpp::Node::SharedPtr nodeHandle;

std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > talon14Publisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > talon15Publisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > talon16Publisher;



enum Error {
    ActuatorsSyncError,
    ActuatorNotMovingError,
    PotentiometerError,
    ConnectionError,
    InitializationError,
    None
};


std::map<Error, const char*> errorMap = {{ActuatorsSyncError, "ActuatorsSyncError"},
    {ActuatorNotMovingError, "ActuatorNotMovingError"},
    {PotentiometerError, "PotentiometerError"},
    {None, "None"}};


struct LinearActuator{
    float speed = 0.0;
    int potentiometer = 0;
    int timeWithoutChange = 0;
    int max = 0;
    int min = 1024;
    Error error = InitializationError;
    bool run = true;
    bool atMin = false;
    bool atMax = false;
    float stroke = 11.8;
};


LinearActuator linear1;
LinearActuator linear2;
LinearActuator linear3;

float speed = 0;
int thresh1 = 60;
int thresh2 = 120;
int thresh3 = 180;

bool automationGo = false;


void sync(){
    float diff = abs(linear1.potentiometer - linear2.potentiometer);
    if(speed > 0){
        if (diff > thresh3){
            (linear1.potentiometer > linear2.potentiometer) ? linear1.speed = 0 : linear2.speed = 0;
            linear1.error = ActuatorsSyncError;
            linear2.error = ActuatorsSyncError;
        }
        else if (diff > thresh2){
            (linear1.potentiometer > linear2.potentiometer) ? linear1.speed *= 0.5 : linear2.speed *= 0.5;
        }
        else if (diff > thresh1){
            (linear1.potentiometer > linear2.potentiometer) ? linear1.speed *= 0.9 : linear2.speed *= 0.9;
        }
        else{
            linear1.error = None;
            linear2.error = None;
        }
    }
    else{
        if (diff > thresh3){
            (linear1.potentiometer < linear2.potentiometer) ? linear1.speed = 0 : linear2.speed = 0;
            linear1.error = ActuatorsSyncError;
            linear2.error = ActuatorsSyncError;
        }
        else if (diff > thresh2){
            (linear1.potentiometer < linear2.potentiometer) ? linear1.speed *= 0.5 : linear2.speed *= 0.5;
        }
        else if (diff > thresh1){
            (linear1.potentiometer < linear2.potentiometer) ? linear1.speed *= 0.9 : linear2.speed *= 0.9;
        }
        else{
            linear1.error = None;
            linear2.error = None;
        }
    }
}


void shoulderCallback(const std_msgs::msg::Float32::SharedPtr speed){
    speed = speed->data;
    if(linear1.error == None || !automationGo)
        linear1.speed = speed->data;
    if(linear2.error == None || !automationGo)
        linear2.speed = speed->data;
    //sync();
    talon14Publisher->publish(linear1.speed);
    talon15Publisher->publish(linear2.speed);
}


void potentiometerCallback(const std_msgs::msg::Int16MultiArray::SharedPtr potent){
    RCLCPP_INFO(nodeHandle->get_logger(),"Potentiometer %d %d", potent->data[0], potent->data[1]);
    
    if(potent->data[0] == 1024){
        linear1.error = PotentiometerError;
    }
    else if(potent->data[0] == -1){
        linear1.error = ConnectionError;
    }
    else{
        if(linear1.error == InitializationError){
            linear1.error = None;
        }
    }

    if(potent->data[1] == 1024){
        linear2.error = PotentiometerError;
    }
    else if(potent->data[1] == -1){
        linear2.error = ConnectionError;
    }
    else{
        if(linear2.error == InitializationError){
            linear2.error = None;
        }
    }

    if(linear1.potentiometer == potent->data[0] && linear1.speed != 0){
        linear1.count += 1;
    }
    else{
        linear1.count = 0;
        linear1.error = None;
    }
    if(potent->data[0] < linear1.min){
        linear1.min = potent->data[0];
    }
    if(potent->data[0] > linear1.max){
        linear1.max = potent->data[0];
    }
    if(linear1.count == 5 && potent->data[0] == linear1.min && linear1.speed < 0){
        linear1.speed = 0;
        linear1.atMax = true;
    }
    if(linear1.count == 5 && potent->data[0] == linear1.max && linear1.speed > 0){
        linear1.speed = 0;
        linear1.atMax = true;
    }
    if(linear1.count == 5 && potent->data[0] != linear1.max && potent->data[0] != linear1.min && linear1.speed != 0){
        linear1.error = PotentiometerError;
        linear2.error = PotentiometerError;
        linear1.speed = 0;
        linear2.speed2 = 0;
    }

    if(linear2.potentiometer == potent->data[1] && linear2.speed != 0){
        linear2.count += 1;
    }
    else{
        linear2.count = 0;
        linear2.error = None;
    }
    if(potent->data[1] < linear2.min){
        linear2.min = potent->data[1];
    }
    if(potent->data[1] > linear2.max){
        linear2.max = potent->data[1];
    }
    if(linear2.count == 5 && potent->data[1] == linear2.min && linear2.speed < 0){
        linear2.speed = 0;
        linear2.atMin = true;
    }
    if(linear2.count == 5 && potent->data[1] == linear2.max && linear2.speed > 0){
        linear2.speed = 0;
        linear2.atMin = true;
    }
    if(linear2.count == 5 && potent->data[1] != linear2.max && potent->data[1] != linear2.min && linear2.speed != 0){
        linear1.error = PotentiometerError;
        linear2.error = PotentiometerError;
        linear1.speed = 0;
        linear2.speed = 0;
    }

    linear1.potentiometer = potent->data[0];
    linear2.potentiometer = potent->data[1];
    //sync();
    
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

    talon14Publisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("talon_14_speed",1);
    talon15Publisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("talon_15_speed",1);
    talon16Publisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("talon_16_speed",1);
    
    messages::msg::LinearOut linearOut1;
    messages::msg::LinearOut linearOut2;
    messages::msg::LinearOut linearOut3;

    auto linearOut1Publisher = nodeHandle->create_publisher<messages::msg::LinearOut>("linearOut1",1);
    auto linearOut2Publisher = nodeHandle->create_publisher<messages::msg::LinearOut>("linearOut2",1);
    auto linearOut3Publisher = nodeHandle->create_publisher<messages::msg::LinearOut>("linearOut3",1);

    rclcpp::Rate rate(20);
    auto start = std::chrono::high_resolution_clock::now();
    while(rclcpp::ok()){
        auto finish = std::chrono::high_resolution_clock::now();
        if(std::chrono::duration_cast<std::chrono::nanoseconds>(finish-start).count() > 250000000){
            linearOut1.speed = linear1.speed;
            linearOut1.potentiometer = linear1.potentiometer;
            linearOut1.timeWithoutChange = linear1.timeWithoutChange;
            linearOut1.max = linear1.max;
            linearOut1.min = linear1.min;
            linearOut1.error = errorMap->linear1.error;
            linearOut1.run = linear1.run;
            linearOut1.at_min = linear1.atMin;
            linearOut1.at_max = linear1.atMin;
            linearOut1Publisher->publish(linearOut1);

            linearOut2.speed = linear2.speed;
            linearOut2.potentiometer = linear2.potentiometer;
            linearOut2.timeWithoutChange = linear2.timeWithoutChange;
            linearOut2.max = linear2.max;
            linearOut2.min = linear2.min;
            linearOut2.error = errorMap->linear2.error;
            linearOut2.run = linear2.run;
            linearOut2.at_min = linear2.atMin;
            linearOut2.at_max = linear2.atMin;
            linearOut2Publisher->publish(linearOut2);

            linearOut3.speed = linear3.speed;
            linearOut3.potentiometer = linear3.potentiometer;
            linearOut3.timeWithoutChange = linear3.timeWithoutChange;
            linearOut3.max = linear3.max;
            linearOut3.min = linear3.min;
            linearOut3.error = errorMap->linear3.error;
            linearOut3.run = linear3.run;
            linearOut3.at_min = linear3.atMin;
            linearOut3.at_max = linear3.atMin;
            linearOut3Publisher->publish(linearOut3);
        }
        rclcpp:spin_some(nodeHandle);
        rate.sleep();
    }
}