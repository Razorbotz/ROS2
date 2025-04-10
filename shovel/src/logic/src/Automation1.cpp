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
        setArmTarget(330);
        setBucketTarget(410);
        setArmSpeed(1.0);
        setBucketSpeed(1.0);
        RCLCPP_INFO(this->node->get_logger(), "linear1.potentiometer: %d", linear1.potentiometer);
        RCLCPP_INFO(this->node->get_logger(), "linear3.potentiometer: %d", linear3.potentiometer);
        robotState = EXCAVATE;
        excavationState = RAISE_ARM;
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

    // TODO: Change this to align
    if(robotState==LOCATE){
        changeSpeed(0.15,-0.15);
        if(position.arucoVisible==true){
            RCLCPP_INFO(this->node->get_logger(), "Roll: %f Pitch: %f Yaw: %f", position.roll, position.pitch, position.yaw);
            setDestAngle(position.yaw + 90.0);
            changeSpeed(0,0);
            robotState=ALIGN;
        }
    }

    // After finding the Aruco marker, turn the bot to 
    // align with the arena
    if(robotState==ALIGN){
        if (!(position.yaw < this->destAngle+2 && position.yaw > this->destAngle-2)) {
            changeSpeed(0.15, -0.15);
        } 
        else {
            changeSpeed(0, 0);
            setStartPosition(this->search.Row - std::ceil(position.z * 10), std::ceil(position.x * 10));
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
    }

    // After aligning with the arena, navigate to the 
    // excavation area
    if(robotState==GO_TO_DIG_SITE){
        RCLCPP_INFO(this->node->get_logger(), "GO_TO_DIG_SITE");
        RCLCPP_INFO(this->node->get_logger(), "ZedPosition.z: %f", this->position.z);
        //TODO: Take rotation of robot into account
        if (!(position.yaw < this->destAngle+2 && position.yaw > this->destAngle-2)) {
            if(position.yaw - this->destAngle > 180 || position.yaw - this->destAngle < 0){
                changeSpeed(0.15, -0.15);
            }
            else{
                changeSpeed(-0.15, 0.15);
            }
        } 
        else{ 
            if(abs(this->position.x) > abs(this->destX)){
                changeSpeed(0.0, 0.0);
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
    if(robotState==GO_TO_HOME){
        if (!(position.yaw < this->destAngle+5 && position.yaw > this->destAngle-5)) {
            if(position.yaw - this->destAngle > 180 || position.yaw - this->destAngle < 0){
                changeSpeed(0.15, -0.15);
            }
            else{
                changeSpeed(-0.15, 0.15);
            }
        } 
        else{ 
            if(abs(this->position.x) > abs(this->destX)){
                changeSpeed(0.0, 0.0);
                if(this->currentPath.empty()){
                    robotState = DOCK;
                }
                else{
                    std::pair<int, int> current = this->currentPath.top();
                    this->currentPath.pop();
                    setDestZ(current.first);
                    setDestX(current.second);
                    setDestAngle(getAngle());
                }
            }
            else if(abs(this->position.x) > abs(this->destX) - 0.1){
                changeSpeed(0.1, 0.1);
            }
            else if(abs(this->position.x) > abs(this->destX) - 0.25){
                changeSpeed(0.15, 0.15);
            }
            else{
                changeSpeed(0.25, 0.25);targetTracking
            }
        }
    }

    // After collecting lunar regolith, align the center of the robot with the corresponding dump zone
    if(robotState==DOCK){
    	
		if (dumpCounter % 4 == 0){ // handles finishing a row of dumps
			xCounter = 0;
			zCounter ++; 
		}
		
		centering(xCounter, zCounter); // Two stage centering on the current dumping site
		
		robotState = DUMP;
    }

    // Dump the collected regolith in the dump zone
    if(robotState==DUMP){
    
        setArmTarget(900);
        setBucketTarget(700);
        setArmSpeed(1.0);
        setBucketSpeed(1.0);
        
        
    	if(checkArmPosition(20) && checkBucketPosition(20))	{				
    		dumpCounter++;		// Keepping track of how many dumps 
    		xCounter++;			// Which coloumn to dump into
    		
    		setArmTarget(100);	// handle bringing the arm and bucket to appropriate driving heights and return to loop
			setBucketTarget(100);
			setArmSpeed(-1.0);
			setBucketSpeed(-1.0);
			
        	robotState = EXCAVATE;
    	
    	}

        

    	
    }

    // After dumping the rocks, return to start position and
    // start again
    if(robotState==RETURN_TO_START){

        robotState = ALIGN;
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
}
    

void Automation1::publishAutomationOut(){
    std::string robotStateString = robotStateMap.at(robotState);
    std::string excavationStateString = excavationStateMap.at(excavationState);
    std::string errorStateString = errorStateMap.at(errorState);
    std::string diagnosticsStateString = diagnosticsStateMap.at(diagnosticsState);
    publishAutonomyOut(robotStateString, excavationStateString, errorStateString, diagnosticsStateString);
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
