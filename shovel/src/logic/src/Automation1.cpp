#include <cmath>
#include <ctime>

#include "logic/Automation.hpp"
#include "logic/Automation1.hpp"

/** @file
 *
 * @brief Defines functions used in Automation1.hpp
 * 
 * This function sets the wheel speed and spins to the right until the camera
 * sees the Aruco marker, then drives forward until the robot is less than a
 * meter away from the marker.
 * */

void Automation1::automate(){
    if(robotState==ROBOT_IDLE){
        //if(deltaX < falcon1.outputPercentage * 0.05 || deltaZ < falcon1.outputPercentage * 0.05){
        //    RCLCPP_INFO(this->node->get_logger(), "ERROR: Robot not moving");
        //}

    }

    if(robotState == INITIAL){
        RCLCPP_INFO(this->node->get_logger(), "Initialize");
        setDestPosition(destX, destY);
        auto start = std::chrono::high_resolution_clock::now();
        setStartTime(start);
        setGo();
        setArmPosition(900);
        setBucketPosition(100);
        if(checkArmPosition(10)){
            robotState = LOCATE;
        }
    }

    if(robotState==DIAGNOSTICS){
        RCLCPP_INFO(this->node->get_logger(), "Diagnostics");
        auto finish = std::chrono::high_resolution_clock::now();
        if(diagnosticsState==TALON_EXTEND){
            setBucketSpeed(1.0);
            setArmSpeed(1.0);
            if(std::chrono::duration_cast<std::chrono::milliseconds>(finish-getStartTime()).count() > 800){
                setBucketSpeed(-1.0);
                setArmSpeed(-1.0);
                if(linear1.error == "ActuatorNotMovingError"){
                    RCLCPP_INFO(this->node->get_logger(), "linear1.error");
                    RCLCPP_INFO(this->node->get_logger(), "%s", linear1.error.c_str());
                    RCLCPP_INFO(this->node->get_logger(), "Talon1.maxCurrent: %f", talon1.maxCurrent);
                    errorState = TALON_14_ERROR;
                    diagnosticsState = DIAGNOSTICS_ERROR_RECOVERY;
                    robotState = ROBOT_IDLE;
                    setBucketSpeed(0.0);
                    setArmSpeed(0.0);
                }
                else if(linear2.error == "ActuatorNotMovingError"){
                    RCLCPP_INFO(this->node->get_logger(), "linear2.error");
                    RCLCPP_INFO(this->node->get_logger(), "%s", linear2.error.c_str());
                    RCLCPP_INFO(this->node->get_logger(), "Talon2.maxCurrent: %f", talon2.maxCurrent);
                    errorState = TALON_15_ERROR;
                    diagnosticsState = DIAGNOSTICS_ERROR_RECOVERY;
                    setBucketSpeed(0.0);
                    setArmSpeed(0.0);
                }
                else if(linear3.error == "ActuatorNotMovingError"){
                    RCLCPP_INFO(this->node->get_logger(), "linear3.error");
                    RCLCPP_INFO(this->node->get_logger(), "%s", linear3.error.c_str());
                    RCLCPP_INFO(this->node->get_logger(), "Talon3.maxCurrent: %f", talon3.maxCurrent);
                    errorState = TALON_16_ERROR;
                    diagnosticsState = DIAGNOSTICS_ERROR_RECOVERY;
                    setBucketSpeed(0.0);
                    setArmSpeed(0.0);
                }
                else if(linear4.error == "ActuatorNotMovingError"){
                    RCLCPP_INFO(this->node->get_logger(), "linear4.error");
                    RCLCPP_INFO(this->node->get_logger(), "%s", linear4.error.c_str());
                    RCLCPP_INFO(this->node->get_logger(), "Talon4.maxCurrent: %f", talon4.maxCurrent);
                    errorState = TALON_17_ERROR;
                    diagnosticsState = DIAGNOSTICS_ERROR_RECOVERY;
                    setBucketSpeed(0.0);
                    setArmSpeed(0.0);
                }
                else{
                    setStartTime(std::chrono::high_resolution_clock::now());
                    diagnosticsState = TALON_RETRACT;
                }
            }
        }
        if(diagnosticsState==TALON_RETRACT){
            RCLCPP_INFO(this->node->get_logger(), "Talon Retract");
            setBucketSpeed(-1.0);
            setArmSpeed(-1.0);
            if(std::chrono::duration_cast<std::chrono::milliseconds>(finish-getStartTime()).count() > 800){
                setBucketSpeed(0.0);
                setArmSpeed(0.0);
                setStartTime(std::chrono::high_resolution_clock::now());
                changeSpeed(0.05, 0.05);
                diagnosticsState = FALCON_FORWARD;
            }
        }
        if(diagnosticsState==FALCON_FORWARD){
            RCLCPP_INFO(this->node->get_logger(), "Falcon Forward");

            if(std::chrono::duration_cast<std::chrono::milliseconds>(finish-getStartTime()).count() > 500){
                changeSpeed(0.0, 0.0);
                if(falcon1.outputCurrent == 0.0){
                    errorState = FALCON_10_ERROR;
                    robotState = ROBOT_IDLE;
                }
                else if(falcon2.outputCurrent == 0.0){
                    errorState = FALCON_11_ERROR;
                    robotState = ROBOT_IDLE;
                }
                else if(falcon3.outputCurrent == 0.0){
                    errorState = FALCON_12_ERROR;
                    robotState = ROBOT_IDLE;
                }
                else if(falcon4.outputCurrent == 0.0){
                    errorState = FALCON_13_ERROR;
                    robotState = ROBOT_IDLE;
                }
                else{
                    diagnosticsState = DIAGNOSTICS_IDLE;
                    robotState = ROBOT_IDLE;
                }
            }
        }
        if(diagnosticsState==DIAGNOSTICS_ERROR_RECOVERY){
            RCLCPP_INFO(this->node->get_logger(), "Error Recovery");
            if(errorState == TALON_14_ERROR){
                
            }
            else if(errorState == TALON_15_ERROR){
            
            }
            else if(errorState == TALON_16_ERROR){

            }
            else if(errorState == TALON_17_ERROR){

            }
            if(std::chrono::duration_cast<std::chrono::milliseconds>(finish-getStartTime()).count() > 500){
                diagnosticsState = DIAGNOSTICS_IDLE;
                robotState = ROBOT_IDLE;
            }
        }
        
    }

    if(robotState==LOCATE){
        // Test function to drive forward a specific number of meters
        /*
        if(abs(this->position.x) > abs(this->destX)){
            changeSpeed(0.0, 0.0);
        }
        else if(abs(this->position.x) > abs(this->destX) - 0.1){
            changeSpeed(0.1, 0.1);
        }
        else if(abs(this->position.x) > abs(this->destX) - 0.25){
            changeSpeed(0.15, 0.15);
        }
        else{
            changeSpeed(0.25, 0.25);
        }
        */

        if(turnLeft){
            changeSpeed(-0.15, 0.15);
        }
        else{
            changeSpeed(0.15, -0.15);
        }
        


        if(position.arucoInitialized==true){
            RCLCPP_INFO(this->node->get_logger(), "Roll: %f Pitch: %f Yaw: %f", position.roll, position.pitch, position.yaw);
            changeSpeed(0,0);
            setStartPositionM(position.z, position.x);
            RCLCPP_INFO(this->node->get_logger(), "startX: %d, startY: %d", this->search.startX, this->search.startY);
            setDestAngle(90);
            robotState=ALIGN;
        }
    }

    // After finding the Aruco marker, turn the bot to 
    // align with the arena
    if(robotState==ALIGN){
        if (!(position.pitch < this->destAngle+2 && position.pitch > this->destAngle-2)) {
            if(getAngleDiff() < 0){
                changeSpeed(-0.15, 0.15);
            }
            else{
                changeSpeed(0.15, -0.15);
            }
        } 
        else {
            changeSpeed(0, 0);
            //setStartPosition(position.z, position.x);
            //aStar();
            //RCLCPP_INFO(this->node->get_logger(), "Current Position: %d, %d", this->search.startX, this->search.startY);
            //std::pair<int, int> initial = this->currentPath.top();
            //this->currentPath.pop();
            //setDestZ(initial.first);
            //setDestX(initial.second);
            //setDestAngle(getAngle());
            setDestX(2.0);
            setDestZ(0.0);
            this->destX = 2.0;
            this->destZ = 0.0;
            robotState = GO_TO_DIG_SITE;
        }
    }

    // After aligning with the arena, navigate to the 
    // excavation area
    if(robotState==GO_TO_DIG_SITE){
        RCLCPP_INFO(this->node->get_logger(), "GO_TO_DIG_SITE");
        RCLCPP_INFO(this->node->get_logger(), "ZedPosition.z: %f", this->position.z);
        if (!(position.pitch < this->destAngle+2 && position.pitch > this->destAngle-2)) {
            if(getAngleDiff() < 0){
                changeSpeed(-0.15, 0.15);
            }
            else{
                changeSpeed(0.15, -0.15);
            }
        } 
        else{ 
            if(abs(this->position.x - this->destX) < 0.05){
                changeSpeed(0.0, 0.0);
                /*
                if(this->currentPath.empty()){
                    robotState = EXCAVATE;
                }
                else{
                    std::pair<int, int> current = this->currentPath.top();
                    this->currentPath.pop();
                    setDestZ(current.first);
                    setDestX(current.second);
                    setDestAngle(getAngle());
                }
                */
                setDestAngle(0);
                setDestX(2.0);
                setDestZ(2.0);
                this->destX = 2.0;
                this->destZ = 2.0;
                robotState = GO_TO_DUMP;
            }
            else if(abs(this->position.x - this->destX) < 0.1){
                changeSpeed(0.1, 0.1);
            }
            else if(abs(this->position.x - this->destX) < 0.25){
                changeSpeed(0.15, 0.15);
            }
            else{
                changeSpeed(0.25, 0.25);
            }
        }
    }

    // After reaching the excavation area, go through mining
    // sequence
    // Check that the potentiometers are in the correct range
    if(robotState==EXCAVATE){
        RCLCPP_INFO(this->node->get_logger(), "EXCAVATE");
        if(excavationState == RAISE_ARM){
            RCLCPP_INFO(this->node->get_logger(), "RAISE_ARM");
            RCLCPP_INFO(this->node->get_logger(), "linear1.potentiometer: %d", linear1.potentiometer);
            RCLCPP_INFO(this->node->get_logger(), "linear3.potentiometer: %d", linear3.potentiometer);
            int armPosition = checkArmPosition(20);
            if(armPosition == 0){
                RCLCPP_INFO(this->node->get_logger(), "RAISE ARM");
                setArmSpeed(1.0);
            }
            if(armPosition == 1){
                RCLCPP_INFO(this->node->get_logger(), "STOP ARM");
                setArmSpeed(0.0);
            }
            if(armPosition == 2){
                RCLCPP_INFO(this->node->get_logger(), "LOWER ARM");
                setArmSpeed(-1.0);
            }
            int bucketPosition = checkBucketPosition(20);
            if(bucketPosition == 0){
                RCLCPP_INFO(this->node->get_logger(), "RAISE BUCKET");
                setBucketSpeed(1.0);
            }
            if(bucketPosition == 1){
                RCLCPP_INFO(this->node->get_logger(), "STOP BUCKET");
                setBucketSpeed(0.0);
            }
            if(bucketPosition == 2){
                RCLCPP_INFO(this->node->get_logger(), "LOWER BUCKET");
                setBucketSpeed(-1.0);
            }
            if(armPosition == 1 && bucketPosition == 1){
                changeSpeed(0.2, 0.2);
                excavationState = COLLECT;
                auto start = std::chrono::high_resolution_clock::now();
                setStartTime(start);
            }
        }
        if(excavationState == COLLECT){
            if(deltaX < falcon1.outputPercentage * 0.05 || deltaZ < falcon1.outputPercentage * 0.05){
                setArmPosition(linear1.potentiometer + 10);
                stillCounter += 1;
                if(stillCounter > 5 || position.pitch > 5){
                    excavationState = EXCAVATION_ERROR_RECOVERY;
                }
            }
            else{
                stillCounter = 0;
            }
            if(deltaX > falcon1.outputPercentage * 0.25 || deltaZ > falcon1.outputPercentage * 0.25){
                setArmPosition(linear1.potentiometer - 10);
            }
            if(abs(this->position.z) > abs(this->destX)){
                changeSpeed(0, 0);
                setArmTarget(900);
                setBucketTarget(850);
                setArmSpeed(1.0);
                setBucketSpeed(1.0);
                excavationState = EXCAVATION_IDLE;
                robotState = DUMP;
            }
            else if(abs(this->position.z) > abs(this->destX) - 0.1){
                changeSpeed(0.1, 0.1);
            }
            else if(abs(this->position.z) > abs(this->destX) - 0.25){
                changeSpeed(0.15, 0.15);
            }
            else{
                changeSpeed(0.25, 0.25);
            }
        }
        if(excavationState == RAISE_BUCKET){

        }
        if(excavationState == LOWER_ARM){

        }
        if(excavationState == LOWER_BUCKET){

        }
        if(excavationState == EXCAVATION_ERROR_RECOVERY){

        }

    }

    // After mining, return to start position
    if(robotState==GO_TO_DUMP){
        if (!(position.pitch < this->destAngle+2 && position.pitch > this->destAngle-2)) {
            if(getAngleDiff() < 0){
                changeSpeed(-0.15, 0.15);
            }
            else{
                changeSpeed(0.15, -0.15);
            }
        } 
        else{ 
            int distance = checkDistance();
            if(distance == 0){
                changeSpeed(0.0, 0.0);
                if(counter == 2){
                    robotState = ROBOT_IDLE;
                }
                else{
                    if(counter == 0){
                        setDestZ(0.0);
                        setDestX(2.0);
                        this->destX = 0.0;
                        this->destZ = 2.0;
                        setDestAngle(-90);
                    }
                    if(counter == 1){
                        setDestZ(0.0);
                        setDestX(0.0);
                        this->destX = 0.0;
                        this->destZ = 0.0;
                        setDestAngle(-180);
                    }
                    counter += 1;
                }
            }
            else if(distance == 1){
                changeSpeed(0.1, 0.1);
            }
            else if(distance == 2){
                changeSpeed(0.15, 0.15);
            }
            else{
                changeSpeed(0.25, 0.25);
            }
        }
    }

    // Dump the collected rocks in the dump bin
    if(robotState==DUMP){
        if(dumpState == DUMP_IDLE){
            setArmTarget(900);
            setBucketTarget(700);
            setArmSpeed(1.0);
            setBucketSpeed(0.0);
            dumpState = DUMP_EXTEND;
        }
        if(dumpState == DUMP_EXTEND){
            if(checkArmPosition(30)){
                setBucketSpeed(1.0);
            }
            if(checkArmPosition(30)){
                setArmSpeed(0.0);
            }
            if(checkBucketPosition(30)){
                setBucketSpeed(0.0);
            }
            if(checkArmPosition(30) && checkBucketPosition(30)){
                setBucketSpeed(-1.0);
                setArmSpeed(-1.0);
                setBucketTarget(10);
                setArmTarget(10);
                dumpState = DUMP_RETRACT;
            }
        }
        if(dumpState == DUMP_RETRACT){
            if(checkArmPosition(30) == 1){
                setArmSpeed(0.0);
            }
            if(checkBucketPosition(30) == 1){
                setBucketSpeed(0.0);
            }
            if(checkArmPosition(30) == 1 && checkBucketPosition(30) == 1){
                robotState = ROBOT_IDLE;
                dumpState = DUMP_IDLE;
            }
        }
        
    }

    if(robotState == OBSTACLE){
        setStartPosition(this->search.Row - std::ceil(position.z * 10), std::ceil(position.x * 10));
        int x = this->search.Row - std::ceil(position.z * 10);
        int y = std::ceil(position.x * 10);
        this->search.setObstacle(x, y, 2);
        aStar();
        RCLCPP_INFO(this->node->get_logger(), "Current Position: %d, %d", this->search.startX, this->search.startY);
        setGo();
        std::pair<int, int> initial = this->currentPath.top();
        this->currentPath.pop();
        setDestZ(initial.first);
        setDestX(initial.second);
        setDestAngle(getAngle());
        robotState = GO_TO_DIG_SITE;
    }

    if(robotState == LEVEL){
        setLevelArms();
        setLevelBucket();
    }
}
    

void Automation1::publishAutomationOut(){
    std::string robotStateString = robotStateMap.at(robotState);
    std::string excavationStateString = excavationStateMap.at(excavationState);
    std::string errorStateString = errorStateMap.at(errorState);
    std::string diagnosticsStateString = diagnosticsStateMap.at(diagnosticsState);
    std::string tiltStateString = tiltStateMap.at(tiltState);
    publishAutonomyOut(robotStateString, excavationStateString, errorStateString, diagnosticsStateString, tiltStateString, std::to_string(levelBucket), std::to_string(levelArms));
}

void Automation1::setDiagnostics(){
    robotState = DIAGNOSTICS;
    diagnosticsState = TALON_EXTEND;
    auto start = std::chrono::high_resolution_clock::now();
    setStartTime(start);
}

void Automation1::startAutonomy(){
    robotState = INITIAL;
    auto start = std::chrono::high_resolution_clock::now();
    setStartTime(start);
}

void Automation1::setLevel(){
    robotState = LEVEL;
    auto start = std::chrono::high_resolution_clock::now();
    setStartTime(start);
}

void Automation1::stopLevel(){
    if(robotState == LEVEL){
        robotState = ROBOT_IDLE;
    }
}