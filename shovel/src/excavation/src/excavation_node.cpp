#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>

#include "messages/msg/linear_out.hpp"

rclcpp::Node::SharedPtr nodeHandle;

std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > talon14Publisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > talon15Publisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > talon16Publisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > talon17Publisher;

messages::msg::LinearOut linearOut1;
messages::msg::LinearOut linearOut2;
messages::msg::LinearOut linearOut3;
messages::msg::LinearOut linearOut4;
/** @file
 * @brief Node to control excavation motors
 * 
 * This node receives information intended to control the
 * linear actuators, then modifies the data to synchronize the
 * two linear actuators attached to the excavation assembly. 
 * This node also sends information about the linear actuators
 * back to the client-side GUI to integrate information about the
 * position data and error state. This node subscribes to the 
 * following topics:
 * \li \b potentiometer_data
 * \li \b shoulder_speed
 * \li \b dump_speed
 * \li \b automationGo
 * 
 * 
 * This node publishes the following topics:
 * \li \b talon_14_speed
 * \li \b talon_15_speed
 * \li \b talon_16_speed
 * \li \b talon_17_speed
 * \li \b linearOut1
 * \li \b linearOut2
 * \li \b linearOut3
 * \li \b linearOut4
 * 
 */


enum Error {
    ActuatorsSyncError,
    ActuatorNotMovingError,
    PotentiometerError,
    ConnectionError,
    None
};


std::map<Error, const char*> errorMap = {{ActuatorsSyncError, "ActuatorsSyncError"},
    {ActuatorNotMovingError, "ActuatorNotMovingError"},
    {PotentiometerError, "PotentiometerError"},
    {ConnectionError, "ConnectionError"},
    {None, "None"}};


struct LinearActuator{
    float speed = 0.0;
    int potentiometer = 0;
    int timeWithoutChange = 0;
    int max = 0;
    int min = 1024;
    int count = 0;
    Error error = ConnectionError;
    bool run = true;
    bool atMin = false;
    bool atMax = false;
    float stroke = 11.8;
    float minExtended = 11.8;
    float maxExtension = 0.0;
};


LinearActuator linear1{0.0, 0, 0, 0, 1024, 0, ConnectionError, true, false, false, 5.9, 5.9, 0.0};
LinearActuator linear2{0.0, 0, 0, 0, 1024, 0, ConnectionError, true, false, false, 5.9, 5.9, 0.0};
LinearActuator linear3{0.0, 0, 0, 0, 1024, 0, ConnectionError, true, false, false, 11.8, 11.8, 0.0};;
LinearActuator linear4{0.0, 0, 0, 0, 1024, 0, ConnectionError, true, false, false, 11.8, 11.8, 0.0};;

float currentSpeed = 0.0;
float currentSpeed2 = 0.0;
int thresh1 = 15;
int thresh2 = 30;
int thresh3 = 45;

bool automationGo = false;


/** @brief Function to sync the linear actuators
 * 
 * The sync function works by checking if the currentSpeed is
 * greater than zero. If the speed is greater than zero, the val
 * checks which actuator is more extended and sets the speed of
 * the actuator to a lower value if the diff is greater than the 
 * thresh values.  If the value is less than zero, the val checks 
 * which actuator is less extended and sets the speed of the 
 * actuator to a lower value. The function then publishes the 
 * updated speed.
 * @return void
 * */
void sync(){
    float diff = abs(linear1.potentiometer - linear2.potentiometer);
    // Might change this from ternary to if statements to improve readability
    bool val = (currentSpeed > 0) ? (linear1.potentiometer > linear2.potentiometer) : (linear1.potentiometer < linear2.potentiometer);
    if (diff > thresh3){
        (val) ? linear1.speed = 0 : linear2.speed = 0;
    }
    else if (diff > thresh2){
        (val) ? linear1.speed *= 0.5 : linear2.speed *= 0.5;
    }
    else if (diff > thresh1){
        (val) ? linear1.speed *= 0.9 : linear2.speed *= 0.9;
    }
    else{
        linear1.speed = currentSpeed;
        linear2.speed = currentSpeed;
    }
}

void sync2(){
    float diff = abs(linear3.potentiometer - linear4.potentiometer);
    // Might change this from ternary to if statements to improve readability
    bool val = (currentSpeed2 > 0) ? (linear3.potentiometer > linear4.potentiometer) : (linear3.potentiometer < linear4.potentiometer);
    if (diff > thresh3){
        (val) ? linear3.speed = 0 : linear4.speed = 0;
    }
    else if (diff > thresh2){
        (val) ? linear3.speed *= 0.5 : linear4.speed *= 0.5;
    }
    else if (diff > thresh1){
        (val) ? linear3.speed *= 0.9 : linear4.speed *= 0.9;
    }
    else{
        linear3.speed = currentSpeed2;
        linear4.speed = currentSpeed2;
    }
}


void setSpeeds(){
    if(!automationGo){
        linear1.speed = currentSpeed;
        linear2.speed = currentSpeed;
    }
    else{
        if(linear1.error != ConnectionError && linear1.error != PotentiometerError && linear2.error != ConnectionError && linear2.error != PotentiometerError){
            linear1.speed = currentSpeed;
            linear2.speed = currentSpeed;
        }
    }
    if(linear1.error != ConnectionError && linear1.error != PotentiometerError && linear2.error != ConnectionError && linear2.error != PotentiometerError){
        sync();
        if(linear1.atMax && currentSpeed > 0){
            linear1.speed = 0.0;
        }
        if(linear2.atMax && currentSpeed > 0){
            linear2.speed = 0.0;
        }
        if(linear1.atMin && currentSpeed < 0){
            linear1.speed = 0.0;
        }
        if(linear2.atMin && currentSpeed < 0){
            linear2.speed = 0.0;
        }
    }
}


void setSpeeds2(){
    if(!automationGo){
        linear3.speed = currentSpeed2;
        linear4.speed = currentSpeed2;
    }
    else{
        if(linear3.error != ConnectionError && linear3.error != PotentiometerError && linear4.error != ConnectionError && linear4.error != PotentiometerError){
            linear3.speed = currentSpeed2;
            linear4.speed = currentSpeed2;
        }
    }
    if(linear3.error != ConnectionError && linear3.error != PotentiometerError && linear4.error != ConnectionError && linear4.error != PotentiometerError){
        sync2();
        if(linear3.atMax && currentSpeed2 > 0){
            linear3.speed = 0.0;
        }
        if(linear4.atMax && currentSpeed2 > 0){
            linear4.speed = 0.0;
        }
        if(linear3.atMin && currentSpeed2 < 0){
            linear3.speed = 0.0;
        }
        if(linear4.atMin && currentSpeed2 < 0){
            linear4.speed = 0.0;
        }
    }
}


/** @brief Function to set potentiometer error
 * 
 * Thsi function is used to set the value of the error
 * of the linear object.  If the potentiometer is equal
 * to 1024, which is the value that occurs when the
 * potentiometer is disconnected from the Arduino. Refer
 * to the ErrorState state diagram for more information.
 * @param potentData - Int value of potentiometer
 * @param *linear - Pointer to linear object
 * @return void
 * */
void setPotentiometerError(int potentData, LinearActuator *linear){
    if(potentData == -1){
        linear->error = ConnectionError;
    }
    else{
        if(linear->error == ConnectionError){
            linear->error = None;
        }
    }
    if(potentData == 1024){
        linear->error = PotentiometerError;
        RCLCPP_INFO(nodeHandle->get_logger(),"EXCAVATION ERROR: PotentiometerError");
    }
    else{
        if(linear->error == PotentiometerError){
            linear->error = None;
        }
    }
}


/** @brief Function to process potentiometer data
 * 
 * This function processes the passed potentiometer data
 * and adjusts the passed linear values accordingly. First
 * the function sets the min and max values if the new data
 * is beyond the previous limits. Next, the function checks
 * if the value is within a threshold of the previous value
 * that is stored in the linear->potentiometer variable. If
 * the value is within this threshold, it's assumed that 
 * the actuator isn't moving. If the speed isn't equal to
 * zero, ie the actuator should be moving, the count
 * variable gets increased. If the count is greater than 5,
 * the function checks if the actuator is at the min or max
 * positions and sets the corresponding values to true if
 * it is.  If the data is outside of the threshold, the 
 * actuator is moving as intended and is not at the min or
 * max positions.
 * @param potentData - Int value of potentiometer
 * @param *linear - Pointer to linear object
 * @return void
 * */
void processPotentiometerData(int potentData, LinearActuator *linear){
    if(potentData < linear->min){
        linear->min = potentData;
    }

    if(potentData > linear->max){
        linear->max = potentData;
    }

    if(linear->potentiometer >= potentData - 10 && linear->potentiometer <= potentData + 10){
        if(linear->speed != 0.0){
            linear->count += 1;
            if(linear->count >= 5){
                if(linear->max > 800 && linear->speed > 0.0 && potentData >= linear->max - 20){
                    linear->atMax = true;
                    linear->count = 0;
                }
                else if(linear->min < 200 && linear->speed < 0.0 && potentData <= linear->min + 20){
                    linear->atMin = true;
                    linear->count = 0;
                }
                else{
                    if(linear->error == None || linear->error == ActuatorsSyncError){
                        linear->error = ActuatorNotMovingError;
                        RCLCPP_INFO(nodeHandle->get_logger(),"EXCAVATION ERROR: ActuatorNotMovingError");
                    }
                }
            }
        }
    }
    else{
        linear->count = 0;
        if(linear->error == ActuatorNotMovingError){
            linear->error = None;
        }
        if(linear->atMax){
            if(linear->speed < 0.0){
                linear->atMax = false;
            }
        }
        else{
            linear->atMax = false;
        }
        if(linear->atMin){
            if(linear->speed > 0.0){
                linear->atMin = false;
            }
        }
        else{
            linear->atMin = false;
        }
    }
    linear->potentiometer = potentData;
}


/** @brief Callback function for the automationGo topic
 * 
 * This function sets the automationGo value to the value
 * in the message.
 * @param msg - ROS2 message containing automationGo value
 * @return void
 * */
void automationGoCallback(const std_msgs::msg::Bool::SharedPtr msg){
    automationGo = msg->data;
}


void setSyncErrors(){
    if(abs(linear1.potentiometer - linear2.potentiometer) > thresh1){
        if(linear1.error == None){
            linear1.error = ActuatorsSyncError;
        }
        if(linear2.error == None){
            linear2.error = ActuatorsSyncError;
        }
    }
    else{
        if(linear1.error == ActuatorsSyncError){
            linear1.error = None;
        }
        if(linear2.error == ActuatorsSyncError){
            linear2.error = None;
        }
    }
    sync();
    if(linear1.speed != 0 || linear2.speed != 0){
        std_msgs::msg::Float32 speed1;
        speed1.data = linear1.speed;
        talon14Publisher->publish(speed1);
        std_msgs::msg::Float32 speed2;
        speed2.data = linear2.speed;
        talon15Publisher->publish(speed2);
    }
}

void setSyncErrors2(){
    if(abs(linear3.potentiometer - linear4.potentiometer) > thresh1){
        if(linear3.error == None){
            linear3.error = ActuatorsSyncError;
        }
        if(linear4.error == None){
            linear4.error = ActuatorsSyncError;
        }
    }
    else{
        if(linear3.error == ActuatorsSyncError){
            linear3.error = None;
        }
        if(linear4.error == ActuatorsSyncError){
            linear4.error = None;
        }
    }
    sync2();
    if(linear3.speed != 0 || linear4.speed != 0){
        std_msgs::msg::Float32 speed1;
        speed1.data = linear3.speed;
        talon16Publisher->publish(speed1);
        std_msgs::msg::Float32 speed2;
        speed2.data = linear4.speed;
        talon17Publisher->publish(speed2);
    }
}


void potentiometer1Callback(const std_msgs::msg::Int32::SharedPtr msg){
    setPotentiometerError(msg->data, &linear1);

    if(linear1.error != ConnectionError && linear1.error != PotentiometerError && linear2.error != ConnectionError && linear2.error != PotentiometerError){
        processPotentiometerData(msg->data, &linear1);
        setSyncErrors();
    }
}


void potentiometer2Callback(const std_msgs::msg::Int32::SharedPtr msg){
    setPotentiometerError(msg->data, &linear2);

    if(linear1.error != ConnectionError && linear1.error != PotentiometerError && linear2.error != ConnectionError && linear2.error != PotentiometerError){
        processPotentiometerData(msg->data, &linear2);
        setSyncErrors();
    }
}


void potentiometer3Callback(const std_msgs::msg::Int32::SharedPtr msg){
    setPotentiometerError(msg->data, &linear3);

    if(linear3.error != ConnectionError && linear3.error != PotentiometerError){
        processPotentiometerData(msg->data, &linear3);
        setSyncErrors2();
    }
}


void potentiometer4Callback(const std_msgs::msg::Int32::SharedPtr msg){
    setPotentiometerError(msg->data, &linear4);

    if(linear4.error != ConnectionError && linear4.error != PotentiometerError){
        processPotentiometerData(msg->data, &linear4);
        setSyncErrors2();
    }
}


void armSpeedCallback(const std_msgs::msg::Float32::SharedPtr speed){
    currentSpeed = speed->data;
    RCLCPP_INFO(nodeHandle->get_logger(),"currentSpeed: %f", currentSpeed);
    if(!automationGo){
        linear1.speed = currentSpeed;
        linear2.speed = currentSpeed;
    }
    else{
        if(linear1.error != ConnectionError && linear1.error != PotentiometerError && linear2.error != PotentiometerError){
            linear1.speed = currentSpeed;
            linear2.speed = currentSpeed;
        }
    }
    if(linear1.error != ConnectionError && linear1.error != PotentiometerError && linear2.error != PotentiometerError){
        sync();
        if(linear1.atMax && currentSpeed > 0){
            linear1.speed = 0.0;
        }
        if(linear2.atMax && currentSpeed > 0){
            linear2.speed = 0.0;
        }
        if(linear1.atMin && currentSpeed < 0){
            linear1.speed = 0.0;
        }
        if(linear2.atMin && currentSpeed < 0){
            linear2.speed = 0.0;
        }
    }
    std_msgs::msg::Float32 speed1;
    speed1.data = linear1.speed;    
    talon14Publisher->publish(speed1);
    std_msgs::msg::Float32 speed2;
    speed2.data = linear2.speed;
    talon15Publisher->publish(speed2);
    RCLCPP_INFO(nodeHandle->get_logger(),"Arm speeds: %f, %f", linear1.speed, linear2.speed);
}


void bucketSpeedCallback(const std_msgs::msg::Float32::SharedPtr speed){
    currentSpeed2 = speed->data;
    RCLCPP_INFO(nodeHandle->get_logger(),"currentSpeed: %f", currentSpeed2);
    if(!automationGo){
        linear3.speed = currentSpeed2;
        linear4.speed = currentSpeed2;
    }
    else{
        if(linear3.error != ConnectionError && linear3.error != PotentiometerError && linear4.error != PotentiometerError){
            linear3.speed = currentSpeed2;
            linear4.speed = currentSpeed2;
        }
    }
    if(linear3.error != ConnectionError && linear3.error != PotentiometerError && linear4.error != PotentiometerError){
        sync2();
        if(linear3.atMax && currentSpeed2 > 0){
            linear3.speed = 0.0;
        }
        if(linear4.atMax && currentSpeed2 > 0){
            linear4.speed = 0.0;
        }
        if(linear3.atMin && currentSpeed2 < 0){
            linear3.speed = 0.0;
        }
        if(linear4.atMin && currentSpeed2 < 0){
            linear4.speed = 0.0;
        }
    }
    std_msgs::msg::Float32 speed1;
    speed1.data = linear3.speed;    
    talon16Publisher->publish(speed1);
    std_msgs::msg::Float32 speed2;
    speed2.data = linear4.speed;
    talon17Publisher->publish(speed2);
    RCLCPP_INFO(nodeHandle->get_logger(),"Bucket speeds: %f, %f", linear3.speed, linear4.speed);

}


/** @brief Function to get the LinearOut values
 * 
 * This function sets the values of the LinearOut message
 * with the values from the linear actuator. 
 * @param *linearOut - Pointer for the LinearOut object
 * @param *linear - Pointer for the linear actuator
 * @return void
 * */
void getLinearOut1(){
    linearOut1.speed = linear1.speed;
    linearOut1.potentiometer = linear1.potentiometer;
    linearOut1.time_without_change = linear1.timeWithoutChange;
    linearOut1.max = linear1.max;
    linearOut1.min = linear1.min;
    linearOut1.error = errorMap.at(linear1.error);
    linearOut1.run = linear1.run;
    linearOut1.at_min = linear1.atMin;
    linearOut1.at_max = linear1.atMax;
}

void getLinearOut2(){
    linearOut2.speed = linear2.speed;
    linearOut2.potentiometer = linear2.potentiometer;
    linearOut2.time_without_change = linear2.timeWithoutChange;
    linearOut2.max = linear2.max;
    linearOut2.min = linear2.min;
    linearOut2.error = errorMap.at(linear2.error);
    linearOut2.run = linear2.run;
    linearOut2.at_min = linear2.atMin;
    linearOut2.at_max = linear2.atMax;
}

void getLinearOut3(){
    linearOut3.speed = linear3.speed;
    linearOut3.potentiometer = linear3.potentiometer;
    linearOut3.time_without_change = linear3.timeWithoutChange;
    linearOut3.max = linear3.max;
    linearOut3.min = linear3.min;
    linearOut3.error = errorMap.at(linear3.error);
    linearOut3.run = linear3.run;
    linearOut3.at_min = linear3.atMin;
    linearOut3.at_max = linear3.atMax;
}

void getLinearOut4(){
    linearOut4.speed = linear4.speed;
    linearOut4.potentiometer = linear4.potentiometer;
    linearOut4.time_without_change = linear4.timeWithoutChange;
    linearOut4.max = linear4.max;
    linearOut4.min = linear4.min;
    linearOut4.error = errorMap.at(linear4.error);
    linearOut4.run = linear4.run;
    linearOut4.at_min = linear4.atMin;
    linearOut4.at_max = linear4.atMax;
}


int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    nodeHandle = rclcpp::Node::make_shared("excavation");

    auto automationGoSubscriber = nodeHandle->create_subscription<std_msgs::msg::Bool>("automationGo",1,automationGoCallback);

    auto armSpeedSubscriber = nodeHandle->create_subscription<std_msgs::msg::Float32>("arm_speed",1,armSpeedCallback);
    auto bucketSpeedSubscriber = nodeHandle->create_subscription<std_msgs::msg::Float32>("bucket_speed",1,bucketSpeedCallback);

    auto potentiometerDataSubscriber1 = nodeHandle->create_subscription<std_msgs::msg::Int32>("potentiometer_1_data",1,potentiometer1Callback);
    auto potentiometerDataSubscriber2 = nodeHandle->create_subscription<std_msgs::msg::Int32>("potentiometer_2_data",1,potentiometer2Callback);
    auto potentiometerDataSubscriber3 = nodeHandle->create_subscription<std_msgs::msg::Int32>("potentiometer_3_data",1,potentiometer3Callback);
    auto potentiometerDataSubscriber4 = nodeHandle->create_subscription<std_msgs::msg::Int32>("potentiometer_4_data",1,potentiometer4Callback);


    talon14Publisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("talon_14_speed",1);
    talon15Publisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("talon_15_speed",1);
    talon16Publisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("talon_16_speed",1);
    talon17Publisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("talon_17_speed",1);
    
    auto linearOut1Publisher = nodeHandle->create_publisher<messages::msg::LinearOut>("linearOut1",1);
    auto linearOut2Publisher = nodeHandle->create_publisher<messages::msg::LinearOut>("linearOut2",1);
    auto linearOut3Publisher = nodeHandle->create_publisher<messages::msg::LinearOut>("linearOut3",1);
    auto linearOut4Publisher = nodeHandle->create_publisher<messages::msg::LinearOut>("linearOut4",1);

    auto start = std::chrono::high_resolution_clock::now();
    while(rclcpp::ok()){
        auto finish = std::chrono::high_resolution_clock::now();
        if(std::chrono::duration_cast<std::chrono::seconds>(finish-start).count() > 2){
            getLinearOut1();
            linearOut1Publisher->publish(linearOut1);

            getLinearOut2();
            linearOut2Publisher->publish(linearOut2);


            getLinearOut3();
            linearOut3Publisher->publish(linearOut3);

            getLinearOut4();
            linearOut4Publisher->publish(linearOut4);
        }
        rclcpp:spin_some(nodeHandle);
    }
}
