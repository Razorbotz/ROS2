#include <cmath>
#include <ctime>
#include "autonomy/Automation2.hpp"

void Automation2::automate() {
    if(robotState == ROBOT_IDLE) {
    }

    if(robotState == INITIAL) {
        RCLCPP_INFO(this->node->get_logger(), "Sisyphus Initialization Sequence");
        auto start = std::chrono::high_resolution_clock::now();
        setStartTime(start);
        setGo();
        
    }

    if(robotState == DIAGNOSTICS) {
        RCLCPP_INFO(this->node->get_logger(), "Running System Pre-Flight Checks");
    }

    if(robotState == LOCATE) {
    }

}

void Automation2::publishAutomationStatus() {
    std::string robotStateString = robotStateMap.at(robotState);
    std::string excavationStateString = excavationStateMap.at(excavationState);
    std::string errorStateString = errorStateMap.at(errorState);
    std::string diagnosticsStateString = diagnosticsStateMap.at(diagnosticsState);
    std::string tiltStateString = tiltStateMap.at(tiltState);
    std::string dumpStateString = dumpStateMap.at(dumpState);
    
    publishAutonomyStatus(robotStateString, excavationStateString, errorStateString, 
                          diagnosticsStateString, tiltStateString, dumpStateString, 
                          std::to_string(levelBucket), std::to_string(levelArms));
}

void Automation2::startAutonomy() {
    robotState = INITIAL;
    auto start = std::chrono::high_resolution_clock::now();
    setStartTime(start);
}

void Automation2::setDiagnostics() {
    robotState = DIAGNOSTICS;
    diagnosticsState = TALON_EXTEND;
    auto start = std::chrono::high_resolution_clock::now();
    setStartTime(start);
}

void Automation2::setLevel() {
    robotState = LEVEL;
    auto start = std::chrono::high_resolution_clock::now();
    setStartTime(start);
}

void Automation2::stopLevel() {
    if(robotState == LEVEL) {
        robotState = ROBOT_IDLE;
    }
}

void Automation2::setDumpMacro() {
    robotState = DUMP_MACRO;
    setGo();
}

void Automation2::setExcavateMacro() {
    robotState = ROBOT_IDLE;
    setGo();
}

void Automation2::setExcavate() {
    currentX = position.x;
    currentZ = position.z;
    excavate = false;
    robotState = ROBOT_IDLE;
}

void Automation2::excavateMacro() {
    if(excavationState == EXCAVATION_IDLE) {
        RCLCPP_INFO(this->node->get_logger(), "Sisyphus collection macro triggered.");
        robotState = ROBOT_IDLE; 
    }
}

void Automation2::dumpMacro() {
    if(dumpState == DUMP_IDLE) {
        RCLCPP_INFO(this->node->get_logger(), "Sisyphus dump macro triggered.");
        robotState = ROBOT_IDLE;
    }
}