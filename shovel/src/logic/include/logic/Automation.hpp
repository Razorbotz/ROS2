#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <messages/msg/linear_out.hpp>
#include <messages/msg/talon_out.hpp>
#include <messages/msg/falcon_out.hpp>
#include <messages/msg/autonomy_out.hpp>
#include <std_msgs/msg/int32.hpp>

#include "AutomationTypes.hpp"
#include "search.hpp"

// Width of the bucket in decimeters
#define BUCKET_WIDTH 7.5

#define TILT_THRESH 10
#define TIP_THRESH 45

// Number of degrees of travel of arm from fully retracted to fully extended
#define ARM_DEGREES 60.0
// Change in potentiometer value from fully retracted to fully extended
#define ARM_TRAVEL 945.0

#define BUCKET_DEGREES 120.0
#define BUCKET_TRAVEL 680.0

class Automation{
    private:
    public:

    enum RobotState{INITIAL,
        DIAGNOSTICS,
        LOCATE,
        ALIGN,
        INITIAL_NAV,
        NAVIGATE,
        EXCAVATE,
        DOCK,
        DUMP,
        OBSTACLE,
        LEVEL,
        ROBOT_IDLE
    };

enum ExcavationState{EXCAVATION_IDLE,
        RAISE_ARM,
        RAISE_BUCKET,
        COLLECT,
        LOWER_ARM,
        LOWER_BUCKET,
        SQUARE_UP,
        DUMP_BUCKET,
        EXCAVATION_ERROR_RECOVERY, 
        RETURN
    };

enum ErrorState {TALON_14_ERROR, 
        TALON_15_ERROR, 
        TALON_16_ERROR, 
        TALON_17_ERROR, 
        FALCON_10_ERROR, 
        FALCON_11_ERROR, 
        FALCON_12_ERROR, 
        FALCON_13_ERROR,
        NONE
    };

enum DiagnosticsState{DIAGNOSTICS_IDLE,
        TALON_EXTEND,
        TALON_RETRACT,
        FALCON_FORWARD,
        DIAGNOSTICS_ERROR_RECOVERY
    };

    enum TiltState{
        TILT_LEVEL,
        TILT_FRONT,
        TILT_FRONT_RIGHT,
        TILT_RIGHT,
        TILT_BACK_RIGHT,
        TILT_BACK,
        TILT_BACK_LEFT,
        TILT_LEFT,
        TILT_FRONT_LEFT,
        TIP_FRONT,
        TIP_BACK,
        TIP_RIGHT,
        TIP_LEFT
    };
    enum DumpState{DUMP_IDLE, 
                    DUMP_EXTEND, 
                    DUMP_RETRACT};
    

    DumpState dumpState = DUMP_IDLE;
    RobotState robotState = ROBOT_IDLE;
    RobotState previousState = ROBOT_IDLE;
    ExcavationState excavationState = EXCAVATION_IDLE;
    ErrorState errorState = NONE;
    DiagnosticsState diagnosticsState = TALON_EXTEND;
    TiltState tiltState = TILT_LEVEL;

    std::map<RobotState, const char*> robotStateMap = {
        {INITIAL, "Initial"},
        {DIAGNOSTICS, "Diagnostics"},
        {LOCATE, "Locate"},
        {ALIGN, "Align"},
        {INITIAL_NAV, "Initial Navigation"},
        {NAVIGATE, "Navigate"},
        {EXCAVATE, "Excavate"},
        {DOCK, "Dock"},
        {DUMP, "Dump"},
        {OBSTACLE, "Obstacle"},
        {LEVEL, "Level"},
        {ROBOT_IDLE,  "Idle"}
    };

    std::map<ExcavationState, const char*> excavationStateMap = {
        {EXCAVATION_IDLE, "Idle"},
        {RAISE_ARM, "Raise Arm"},
        {RAISE_BUCKET, "Raise Bucket"},
        {COLLECT, "Collect"},
        {DUMP_BUCKET, "Dump"},
        {LOWER_ARM, "Lower Arm"},
        {LOWER_BUCKET, "Lower Bucket"},
        {SQUARE_UP, "Square Up"},
        {EXCAVATION_ERROR_RECOVERY, "Error Recovery"}, 
        {RETURN, "Return"}
    };

    std::map<ErrorState, const char*> errorStateMap = {
        {TALON_14_ERROR, "Talon 14 Error"},
        {TALON_15_ERROR, "Talon 15 Error"},
        {TALON_16_ERROR, "Talon 16 Error"},
        {TALON_17_ERROR, "Talon 17 Error"},
        {FALCON_10_ERROR, "Falcon 10 Error"},
        {FALCON_11_ERROR, "Falcon 11 Error"},
        {FALCON_12_ERROR, "Falcon 12 Error"},
        {FALCON_13_ERROR, "Falcon 13 Error"},
        {NONE, "None"}
    };

    std::map<DiagnosticsState, const char*> diagnosticsStateMap = {
        {DIAGNOSTICS_IDLE, "Idle"},
        {TALON_EXTEND, "Talon Extend"},
        {TALON_RETRACT, "Talon Retract"},
        {FALCON_FORWARD, "Falcon Forward"},
        {DIAGNOSTICS_ERROR_RECOVERY, "Error Recovery"}
    };

    std::map<TiltState, const char*> tiltStateMap = {
        {TILT_LEVEL, "Level"},
        {TILT_FRONT, "Front"},
        {TILT_FRONT_RIGHT, "Front Right"},
        {TILT_RIGHT, "Right"},
        {TILT_BACK_RIGHT, "Back Right"},
        {TILT_BACK, "Back"},
        {TILT_BACK_LEFT, "Back Left"},
        {TILT_LEFT, "Left"},
        {TILT_FRONT_LEFT, "Front Left"},
        {TIP_FRONT, "Tip Front"},
        {TIP_BACK, "Tip Back"},
        {TIP_RIGHT, "Tip Right"},
        {TIP_LEFT, "Tip Left"}
    };

    std::map<DumpState, const char*> dumpStateMap = {
        {DUMP_IDLE, "Idle"},
        {DUMP_EXTEND, "Extend"},
        {DUMP_RETRACT, "Retract"},
    };

    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > driveLeftSpeedPublisher;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > driveRightSpeedPublisher;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > armSpeedPublisher;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > bucketSpeedPublisher;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Empty_<std::allocator<void> >, std::allocator<void> > > goPublisher;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Empty_<std::allocator<void> >, std::allocator<void> > > stopPublisher;
    std::shared_ptr<rclcpp::Publisher<messages::msg::AutonomyOut_<std::allocator<void> >, std::allocator<void> > > autonomyOutPublisher;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int32_<std::allocator<void> >, std::allocator<void> > > talon14PositionPublisher;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int32_<std::allocator<void> >, std::allocator<void> > > talon15PositionPublisher;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int32_<std::allocator<void> >, std::allocator<void> > > talon16PositionPublisher;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int32_<std::allocator<void> >, std::allocator<void> > > talon17PositionPublisher;

    rclcpp::Node::SharedPtr node;
    Position position;
    Quaternion orientationQuaternion;
    EulerAngles orientation;
    float currentLeftSpeed=0;
    float currentRightSpeed=0;
    Linear linear1, linear2, linear3, linear4;
    MotorOut talon1, talon2, talon3, talon4, falcon1, falcon2, falcon3, falcon4;
    // Height, width, excavation area, obstacle area, target Area
    Arena NASA{50, 69, 20, 39, {0, 0, 0, 0}};
    Arena UCF_1{82, 46, 1, 41, {0, 0, 0, 0}};
    Arena UCF_2{82, 46, 1, 41, {0, 0, 0, 0}};
    Arena lab{40, 50, 20, 25, {0, 0, 0, 0}};
    float destX = 0, destZ = 0, destAngle=0;
    float prevX = 0.0, prevY = 0.0, prevZ = 0.0;
    float deltaX = 0.0, deltaY = 0.0, deltaZ = 0.0;
    float prevDist = 0.0;
    int target1 = 0, target3 = 0;
    float robotWidth = 7.5;
    float angleThresh = 1.5;
    int targetTracking[4][int(std::floor(20/BUCKET_WIDTH))] = {0};
    int dumpCounter = 0, xCounter = 0, zCounter = 0;
    std::stack<Coord> currentPath;
    std::chrono::time_point<std::chrono::high_resolution_clock> startTime;
    Search search = Search();
    bool holes = false;
    bool turnLeft = true;

    bool runSensorlessly = false;

    bool levelBucket = true;
    bool levelArms = true;
    bool centeringSecond = false;
    float currentX = 0.0, currentZ = 0;
    bool dump = true;
    bool excavate = true;

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

    void setIdle();

    virtual void setDiagnostics() = 0;
    
    virtual void startAutonomy() = 0; 

    virtual void setLevel() = 0;

    virtual void stopLevel() = 0;

    virtual void dumpMacro() = 0;

    virtual void excavateMacro() = 0;

    virtual void setDump() = 0;

    virtual void setExcavate() = 0;
    
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

    float getAngle2();

    float getAngleDiff(float degrees);

    float getAngleDiff();

    bool checkAngle(bool reverse = false);

    bool checkAngle2();

    int checkDistance(float thresh);

    bool driveToTarget(float closeThresh);

    void setDestX(float meters);

    void setDestZ(float meters);

    bool getPosition();

    void publishAutonomyOut(std::string robotStateString, std::string excavationStateString, std::string errorStateString, std::string diagnosticsStateString, std::string tiltStateString, std::string dumpStateString, std::string bucketState, std::string armsState);

    void setStartTime(std::chrono::time_point<std::chrono::high_resolution_clock> StartTime);

    std::chrono::time_point<std::chrono::high_resolution_clock> getStartTime();

    void setRunSensorlessly(bool value);

    void setStartPosition(float x, float y);

    void setDestPosition(float x, float y);

    void aStar(bool includeHoles = false);

    void aStarStack(bool includeHoles, bool simplify);

    void addPointToStack(float x, float z);

    int checkArmPosition(int thresh);

    int checkBucketPosition(int thresh);

    void setArmPosition(int potent);

    void setBucketPosition(int potent);

    void setMap(std::string mapUsed);

    void setTurnLeft(bool TurnLeft);

    void centering(int xCounter, int zCounter);

    enum TiltState checkOrientation();

    void setLevelBucket();

    void setLevelArms();

    void startBucketLevel();
    
    void startArmsLevel();

    void stopBucketLevel();

    void stopArmsLevel();
};
