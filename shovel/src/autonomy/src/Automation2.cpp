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
    diagnosticsState = TALON_EXTEND; // Or Sisyphus equivalent
    auto start = std::chrono::high_resolution_clock::now();
    setStartTime(start);
}