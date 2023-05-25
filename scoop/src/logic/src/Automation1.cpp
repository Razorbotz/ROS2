#include <cmath>
#include <ctime>

#include "logic/Automation.hpp"
#include "logic/Automation1.hpp"

int left = 0;

/** @file
 *
 * @brief Defines functions used in Automation1.hpp
 * 
 * This function sets the wheel speed and spins to the right until the camera
 * sees the Aruco marker, then drives forward until the robot is less than a
 * meter away from the marker.
 * */

void Automation1::automate(){
    // Initially start with locating the Aruco marker
    // Turn slowly until it's seen
    if(robotState==LOCATE){
        changeSpeed(0.15,-0.15);
        if(position.arucoVisible==true){
            if (abs(position.aruco_roll) < 90.0) {
                left = 1;
            } else { // dot on top
                left = -1;
            }
            RCLCPP_INFO(this->node->get_logger(), "Left: %d", left);
            setDestAngle(position.yaw + 90.0);
            destination.x=-2;
            destination.z=1;
            changeSpeed(0,0);
            robotState=ALIGN;
        }
    }
/*  
    if(robotState==GO_TO_DIG_SITE){
        double yawRadians=this->orientation.roll;

        double facingUnitX=-sin(yawRadians);
        double facingUnitZ=cos(yawRadians);
        double directionX=destination.x-position.x;
        double directionZ=destination.z-position.z;

        double theta = acos((facingUnitX*directionX + facingUnitZ*directionZ)/(sqrt(directionX*directionX + directionZ*directionZ)))*180/M_PI;
        double yaw = yawRadians * 180/M_PI;
        double deltaYaw = theta-yaw;
        double yawTolerance=5;
        if(deltaYaw > yawTolerance){
            changeSpeed(-0.15,0.15);
        }
        else if (deltaYaw < yawTolerance){
            changeSpeed(0.15,-0.15);
        }
        else{
            changeSpeed(0.15 - 0.1*deltaYaw/yawTolerance,0.15 + 0.1*deltaYaw/yawTolerance);
        }
        std::cout << orientation.roll*180/M_PI << ", " << orientation.pitch*180/M_PI << ", " << orientation.yaw*180/ M_PI << "   "
                << "   \t" << position.x << "  " << position.y << "  " << position.z
                << "   \t" << position.ox << "  " << position.oy << "  " << position.oz << "  " << position.ow
                << "   \t" << facingUnitX << " " << facingUnitZ << "   " << yaw << " " << deltaYaw << " " << theta
              << "   \t" << position.arucoVisible << std::endl;
    }
*/    

    // After finding the Aruco marker, turn the bot to 
    // align with the arena
    if(robotState==ALIGN){
        RCLCPP_INFO(this->node->get_logger(), "Left: %d", left);
        if (!(position.yaw < this->destAngle+5 && position.yaw > this->destAngle-5)) {
            changeSpeed(0.15*left, -0.15*left);
        } else {
            changeSpeed(0, 0);
            setDestDistance(position.x - 1.0);
            setGo();
            changeSpeed(0.25, 0.25);
            robotState = GO_TO_DIG_SITE;
        }
    }

    // After aligning with the arena, navigate to the 
    // excavation area
    if(robotState==GO_TO_DIG_SITE){
        RCLCPP_INFO(this->node->get_logger(), "GO_TO_DIG_SITE");
        RCLCPP_INFO(this->node->get_logger(), "ZedPosition.z: %f", this->position.z);
        RCLCPP_INFO(this->node->get_logger(), "Left: %d", left);
        if(abs(this->position.x) > abs(this->destDistance)){
            changeSpeed(0.0, 0.0);
            robotState = EXCAVATE;
        }
        else if(abs(this->position.x) > abs(this->destDistance) - 0.1){
            changeSpeed(0.1, 0.1);
        }
        else if(abs(this->position.x) > abs(this->destDistance) - 0.25){
            changeSpeed(0.15, 0.15);
        }
        else{
            changeSpeed(0.25, 0.25);
        }
    }

    // After reaching the excavation area, go through mining
    // sequence
    if(robotState==EXCAVATE){
        if(excavationState == EXCAVATION_IDLE){
            RCLCPP_INFO(this->node->get_logger(), "EXCAVATION AUTONOMY: EXCAVATION_IDLE STATE");
            excavationState = LOWER_ASSEMBLY;
            auto start = std::chrono::high_resolution_clock::now();
            setStartTime(start);
        }
        
        if(excavationState == LOWER_ASSEMBLY){
            RCLCPP_INFO(this->node->get_logger(), "EXCAVATION AUTONOMY: LOWER_ASSEMBLY STATE");
            setShoulderSpeed(0.8);
            
            if(!runSensorlessly){
                if(checkErrors(linear1) || checkErrors(linear2)){
                    excavationState = ERROR_RECOVERY;
                    errorState = LOWER_ASSEMBLY_ERROR;
                }

                if(linear1.atMax && linear2.atMax){
                    setShoulderSpeed(0.0);
                    setNeoSpeed(neoSpeed);
                    setStepperSpeed(1);
                    auto start = std::chrono::high_resolution_clock::now();
                    setStartTime(start);
                    excavationState = LOWER_LADDER;
                }
            }
            else{
                auto finish = std::chrono::high_resolution_clock::now();
                if(std::chrono::duration_cast<std::chrono::seconds>(finish-getStartTime()).count() > 25){
                    setShoulderSpeed(0.0);
                    setNeoSpeed(neoSpeed);
                    setStepperSpeed(1);
                    auto start = std::chrono::high_resolution_clock::now();
                    setStartTime(start);
                    excavationState = LOWER_LADDER;
                }
            }
        }

        //Lower ladder
        if(excavationState == LOWER_LADDER){
            RCLCPP_INFO(this->node->get_logger(), "EXCAVATION AUTONOMY: LOWER_LADDER STATE");
            auto finish = std::chrono::high_resolution_clock::now();
            if(std::chrono::duration_cast<std::chrono::seconds>(finish-getStartTime()).count() > (extensionDuration)){
                auto start = std::chrono::high_resolution_clock::now();
                setStartTime(start);
                setBackupStartTime(start);
                setStepperSpeed(0);
                setGo();
                excavationState = DIG;
            }
        }

        //Dig
        // Have ladder fully extended and spinning
        // Back up robot to get more regolith
        if(excavationState == DIG){
            RCLCPP_INFO(this->node->get_logger(), "EXCAVATION AUTONOMY: DIG STATE");
            auto finish = std::chrono::high_resolution_clock::now();
            if(std::chrono::duration_cast<std::chrono::seconds>(finish-getBackupStartTime()).count() > (stopDuration)){
                if(std::chrono::duration_cast<std::chrono::seconds>(finish-getBackupStartTime()).count() > (stopDuration + reverseDuration)){
                    setBackupStartTime(finish);
                    changeSpeed(0, 0);
                }
                else{
                    changeSpeed(reverseSpeed, reverseSpeed);
                }
            }
		    if(std::chrono::duration_cast<std::chrono::seconds>(finish-getStartTime()).count() > (excavationDuration)){
                auto start = std::chrono::high_resolution_clock::now();
                setStartTime(start);
                changeSpeed(0, 0);
                setNeoSpeed(0.0);
                excavationState = RAISE_ASSEMBLY;
            }
        }
        if(excavationState == RAISE_ASSEMBLY){
            RCLCPP_INFO(this->node->get_logger(), "EXCAVATION AUTONOMY: RAISE_ASSEMBLY STATE");
            setShoulderSpeed(-0.8);
            
            if(!runSensorlessly){
                if(checkErrors(linear1) || checkErrors(linear2)){
                    excavationState = ERROR_RECOVERY;
                    errorState = RAISE_ASSEMBLY_ERROR;
                }

                if(linear1.atMin && linear2.atMin){
                    // setShoulderSpeed(0.0);
                    // excavationState = EXCAVATION_IDLE;
                    // destination.x=0;
                    // destination.z=0;
                    // setDestDistance(1.0);
                    // setDestAngle(position.yaw + left*180.0);
                    // robotState = GO_TO_HOME;

                    setShoulderSpeed(0.0);
                    excavationState = RAISE_LADDER;
                    //destination.x=0;
                    //destination.z=0;
                    //setDestDistance(1.0);
                    //double adjPos = position.yaw - 180.0;
                    //if (adjPos < -180.0) {
                    //    adjPos = abs(adjPos) - 2.0 * (adjPos + 180);
                    //}
                    //setDestAngle(adjPos);
                    robotState = EXCAVATE;
                    setStepperSpeed(-1);
                    auto start = std::chrono::high_resolution_clock::now();
                    setStartTime(start);
                }
            }
            else{
                auto finish = std::chrono::high_resolution_clock::now();
                if(std::chrono::duration_cast<std::chrono::seconds>(finish-getStartTime()).count() > 25){
                    setShoulderSpeed(0.0);
                    excavationState = RAISE_LADDER;
                    robotState = EXCAVATE;
                    setStepperSpeed(-1);
                    auto start = std::chrono::high_resolution_clock::now();
                    setStartTime(start);
                }
            }
        }
        //Raise ladder
        if(excavationState == RAISE_LADDER){
            RCLCPP_INFO(this->node->get_logger(), "EXCAVATION AUTONOMY: RAISE_LADDER STATE");
            auto finish = std::chrono::high_resolution_clock::now();
		    if(std::chrono::duration_cast<std::chrono::seconds>(finish-getStartTime()).count() > (extensionDuration)){
                excavationState = EXCAVATION_IDLE;
                setStepperSpeed(0);
            }
        }

        //Raise assembly
        

        //Handle errors
        // If Connection error, fail
        // If potentiometer error, fail
        if(excavationState == ERROR_RECOVERY){
            RCLCPP_INFO(this->node->get_logger(), "EXCAVATION AUTONOMY: ERROR_RECOVERY STATE");
            if(errorState == LOWER_ASSEMBLY_ERROR || errorState == RAISE_ASSEMBLY_ERROR){
                if(linear1.error == "ActuatorNotMovingError" || linear2.error == "ActuatorNotMovingError"){
                    // Move linear actuators down and up
                    std_msgs::msg::Float32 speed;
                    speed.data = -0.8;
                    shoulderPublisher->publish(speed);
                    auto start = std::chrono::high_resolution_clock::now();
                    auto finish = std::chrono::high_resolution_clock::now();
                    while(std::chrono::duration_cast<std::chrono::nanoseconds>(finish-start).count() < 2000000000){
                        finish = std::chrono::high_resolution_clock::now();
                    }
                    speed.data = 0.8;
                    shoulderPublisher->publish(speed);
                    start = std::chrono::high_resolution_clock::now();
                    finish = std::chrono::high_resolution_clock::now();
                    while(std::chrono::duration_cast<std::chrono::nanoseconds>(finish-start).count() < 2000000000){
                        finish = std::chrono::high_resolution_clock::now();
                    }
                    if(linear1.error == "None" && linear2.error == "None"){
                        if(errorState == LOWER_ASSEMBLY_ERROR){
                            excavationState = LOWER_ASSEMBLY;
                        }
                        if(errorState == RAISE_ASSEMBLY_ERROR){
                            excavationState = RAISE_ASSEMBLY;
                        }
                        errorState = NONE;
                    }
                    // If it doesn't move, break
                    else{
                        RCLCPP_INFO(this->node->get_logger(), "EXCAVATION AUTONOMY ERROR: AcutatorNotMovingError. Ending Autonomy.");
                        excavationState = EXCAVATION_IDLE;
                        robotState = ROBOT_IDLE;
                        std_msgs::msg::Float32 speed;
                        speed.data = 0.8;
                        shoulderPublisher->publish(speed);
                    }
                }
                else if(linear1.error == "None" && linear2.error == "None"){
                    if(errorState == LOWER_ASSEMBLY_ERROR){
                        excavationState = LOWER_ASSEMBLY;
                    }
                    if(errorState == RAISE_ASSEMBLY_ERROR){
                        excavationState = RAISE_ASSEMBLY;
                    }
                    errorState = NONE;
                }
                else{
                    RCLCPP_INFO(this->node->get_logger(), "EXCAVATION AUTONOMY ERROR: PotentiometerError or ConnectionError. Ending Autonomy.");
                    excavationState = EXCAVATION_IDLE;
                    robotState = ROBOT_IDLE;
                    setStop();
                    std_msgs::msg::Float32 speed;
                    speed.data = 0.8;
                    shoulderPublisher->publish(speed);
                }
            }
        }
    }

    // After mining, return to start position
    if(robotState==GO_TO_HOME){
        if (!(position.yaw < this->destAngle+5 && position.yaw > this->destAngle-5)) {
            changeSpeed(0.15, -0.15);
        }
        else if(abs(this->position.x) > abs(this->destDistance)){
            changeSpeed(0.0, 0.0);
            robotState = DOCK;
        }
        else if(abs(this->position.x) > abs(this->destDistance) - 0.1){
            changeSpeed(0.1, 0.1);
        }
        else if(abs(this->position.x) > abs(this->destDistance) - 0.25){
            changeSpeed(0.15, 0.15);
        }
        else{
            changeSpeed(0.25, 0.25);
        }
    }

    // After reaching start position, dock at dump bin
    if(robotState==DOCK){

        robotState = DUMP;
    }

    // Dump the collected rocks in the dump bin
    if(robotState==DUMP){
        if(dumpState == DUMP_IDLE){
            dumpState = DUMP_LOWER_ASSEMBLY;
        }
        
        if(dumpState == DUMP_LOWER_ASSEMBLY){
            RCLCPP_INFO(this->node->get_logger(), "DUMP AUTONOMY: DUMP_LOWER_ASSEMBLY STATE");
            setShoulderSpeed(0.8);
                
            if(checkErrors(linear1) || checkErrors(linear2)){
                dumpState = DUMP_ERROR_RECOVERY;
                errorState = LOWER_ASSEMBLY_ERROR;
            }

            if(linear1.atMax && linear2.atMax){
                setShoulderSpeed(0.0);
                dumpState = RAISE_BIN;
            }
        }

        if(dumpState == RAISE_BIN){
            RCLCPP_INFO(this->node->get_logger(), "DUMP AUTONOMY: RAISE_BIN STATE");
            setDumpSpeed(0.8);
            
            if(checkErrors(linear3)){
                dumpState = DUMP_ERROR_RECOVERY;
                errorState = RAISE_BIN_ERROR;
            }

            if(linear3.atMax){
                setDumpSpeed(0.0);
                dumpState = SERVO;
            }
        }

        if(dumpState == SERVO){
            dumpState = LOWER_BIN;
        }

        if(dumpState == LOWER_BIN){
            RCLCPP_INFO(this->node->get_logger(), "DUMP AUTONOMY: LOWER_BIN STATE");
            setDumpSpeed(-0.8);
            
            if(checkErrors(linear3)){
                dumpState = DUMP_ERROR_RECOVERY;
                errorState = LOWER_BIN_ERROR;
            }

            if(linear3.atMin){
                setDumpSpeed(0.0);
                dumpState = DUMP_RAISE_ASSEMBLY;
            }
        }

        if(dumpState == DUMP_RAISE_ASSEMBLY){
            RCLCPP_INFO(this->node->get_logger(), "DUMP AUTONOMY: DUMP_RAISE_ASSEMBLY STATE");
            setShoulderSpeed(-0.8);
                
            if(checkErrors(linear1) || checkErrors(linear2)){
                dumpState = DUMP_ERROR_RECOVERY;
                errorState = RAISE_ASSEMBLY_ERROR;
            }

            if(linear1.atMax && linear2.atMax){
                setShoulderSpeed(0.0);
                dumpState = DUMP_IDLE;
                robotState = RETURN_TO_START;
            }
        }
    }

    // After dumping the rocks, return to start position and
    // start again
    if(robotState==RETURN_TO_START){

        robotState = ALIGN;
    }
}
    

void Automation1::publishAutomationOut(){
    std::string robotStateString = robotStateMap.at(robotState);
    std::string excavationStateString = excavationStateMap.at(excavationState);
    std::string errorStateString = errorStateMap.at(errorState);
    std::string dumpStateString = dumpStateMap.at(dumpState);
    publishAutonomyOut(robotStateString, excavationStateString, errorStateString, dumpStateString);
}