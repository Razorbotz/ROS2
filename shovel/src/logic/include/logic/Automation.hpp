#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <messages/msg/linear_out.hpp>
#include <messages/msg/talon_out.hpp>
#include <messages/msg/falcon_out.hpp>
#include <messages/msg/autonomy_out.hpp>

#include "AutomationTypes.hpp"
#include "search.hpp"

// Width of the bucket in decimeters
#define BUCKET_WIDTH 7.5

class Automation{
    private:
    public:

    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > driveLeftSpeedPublisher;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > driveRightSpeedPublisher;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > armSpeedPublisher;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > bucketSpeedPublisher;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Empty_<std::allocator<void> >, std::allocator<void> > > goPublisher;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Empty_<std::allocator<void> >, std::allocator<void> > > stopPublisher;
    std::shared_ptr<rclcpp::Publisher<messages::msg::AutonomyOut_<std::allocator<void> >, std::allocator<void> > > autonomyOutPublisher;

    rclcpp::Node::SharedPtr node;
    Position position;
    Quaternion orientationQuaternion;
    EulerAngles orientation;
    float currentLeftSpeed=0;
    float currentRightSpeed=0;
    Linear linear1, linear2, linear3, linear4;
    MotorOut talon1, talon2, talon3, talon4, falcon1, falcon2, falcon3, falcon4;
    Arena NASA{69, 50, 20, 39, {0, 0, 0, 0}};
    Arena UCF_1{46, 82, 1, 41, {0, 0, 0, 0}};
    Arena UCF_2{46, 82, 1, 41, {0, 0, 0, 0}};
    Arena lab{40, 50, 20, 25, {0, 0, 0, 0}};
    float destX = 0, destZ = 0, destAngle=0;
    float prevX = 0.0, prevY = 0.0, prevZ = 0.0;
    float deltaX = 0.0, deltaY = 0.0, deltaZ = 0.0;
    int target1 = 0, target3 = 0;
    float xOffset = 0.0;
    float robotWidth = 7.5;
    int targetTracking[4][int(std::floor(20/BUCKET_WIDTH))] = {0};
    int dumpCounter = 0, xCounter = 0, yCounter = 0;
    std::stack<Coord> currentPath;
    std::chrono::time_point<std::chrono::high_resolution_clock> startTime;
    std::chrono::time_point<std::chrono::high_resolution_clock> startBackupTime;
    Search search = Search();
    bool holes = false;
    bool turnLeft = true;

    bool runSensorlessly = false;

    virtual void automate() = 0;

    virtual void publishAutomationOut() = 0;

    void setNode(rclcpp::Node::SharedPtr node);

    void setPosition(Position position);

    void changeSpeed(float left, float right);

    void setBucketSpeed(float speed);

    void setArmSpeed(float speed);

    EulerAngles toEulerAngles(Quaternion q); 

    void setGo();

    void setStop();

    virtual void setDiagnostics() = 0;
    
    virtual void startAutonomy() = 0; 
    
    void stopActuators();

    void setLinear1(const messages::msg::LinearOut::SharedPtr linearOut);

    void setLinear2(const messages::msg::LinearOut::SharedPtr linearOut);

    void setLinear3(const messages::msg::LinearOut::SharedPtr linearOut);

    void setLinear4(const messages::msg::LinearOut::SharedPtr linearOut);

    void setTalon1(const messages::msg::TalonOut::SharedPtr talonOut);

    void setTalon2(const messages::msg::TalonOut::SharedPtr talonOut);

    void setTalon3(const messages::msg::TalonOut::SharedPtr talonOut);

    void setTalon4(const messages::msg::TalonOut::SharedPtr talonOut);

    void setFalcon1(const messages::msg::FalconOut::SharedPtr falconOut);
    
    void setFalcon2(const messages::msg::FalconOut::SharedPtr falconOut);
    
    void setFalcon3(const messages::msg::FalconOut::SharedPtr falconOut);
    
    void setFalcon4(const messages::msg::FalconOut::SharedPtr falconOut);

    bool checkErrors(Linear linear);

    void setDestAngle(float degrees);

    float getAngle();

    float getAngleDiff(float degrees);

    int checkAngle();

    int checkDistance();

    void setDestX(float meters);

    void setDestZ(float meters);

    void publishAutonomyOut(std::string robotStateString, std::string excavationStateString, std::string errorStateString, std::string dumpStateString);

    void setStartTime(std::chrono::time_point<std::chrono::high_resolution_clock> StartTime);

    void setBackupStartTime(std::chrono::time_point<std::chrono::high_resolution_clock> StartTime);

    std::chrono::time_point<std::chrono::high_resolution_clock> getStartTime();

    std::chrono::time_point<std::chrono::high_resolution_clock> getBackupStartTime();

    void setRunSensorlessly(bool value);

    void setStartPositionM(float x, float y);

    void setStartPosition(int x, int y);

    void setDestPositionM(float x, float y);

    void setDestPosition(int x, int y);

    void aStar(bool includeHoles = false);

    void aStar(std::stack<Coord> points, bool includeHoles, bool simplify);

    void setArmTarget(int potent);

    void setBucketTarget(int potent);

    int checkArmPosition(int thresh);

    int checkBucketPosition(int thresh);

    void setArmPosition(int potent);

    void setBucketPosition(int potent);

    void setMap(std::string mapUsed);

    void setxOffset(float xOffset);

    void setTurnLeft(bool TurnLeft);
};
