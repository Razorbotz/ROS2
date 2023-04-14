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
            // robotState = EXCAVATE;
            setShoulderSpeed(0.0);
            // excavationState = IDLE;
            destination.x=0;
            destination.z=0;
            setDestDistance(1.0);
            setDestAngle(position.yaw - 180.0);
            robotState = GO_TO_HOME;
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
        robotState=GO_TO_HOME;
        // if(excavationState == IDLE){
        //     RCLCPP_INFO(this->node->get_logger(), "EXCAVATION AUTONOMY: IDLE STATE");
        //     excavationState = LOWER_ASSEMBLY;
        // }
        
        // if(excavationState == LOWER_ASSEMBLY){
        //     RCLCPP_INFO(this->node->get_logger(), "EXCAVATION AUTONOMY: LOWER_ASSEMBLY STATE");
        //     setShoulderSpeed(0.8);
            
        //     if(checkErrors(linear1) || checkErrors(linear2)){
        //         excavationState = ERROR_RECOVERY;
        //         errorState = LOWER_ASSEMBLY_ERROR;
        //     }

        //     if(linear1.atMax && linear2.atMax){
        //         setShoulderSpeed(0.0);
        //         excavationState = LOWER_LADDER;
        //     }
        // }

        // //Lower ladder
        //     //Set speed to 0.4
            
        //     //Check for errors
            
        //     //Handle errors
            
        //     //atMax, move to dig
        // if(excavationState == LOWER_LADDER){
        //     RCLCPP_INFO(this->node->get_logger(), "EXCAVATION AUTONOMY: LOWER_LADDER STATE");
        //     setNeoSpeed(0.1);
        //     excavationState = DIG;
        // }

        // //Dig
        // // Have ladder fully extended and spinning
        // // Back up robot to get more regolith
        // if(excavationState == DIG){
        //     RCLCPP_INFO(this->node->get_logger(), "EXCAVATION AUTONOMY: DIG STATE");
        //     excavationState = RAISE_LADDER;
        // }
        
        // //Raise ladder
        //     //Set speed to -0.4
            
        //     //Check for errors
            
        //     //Handle errors
            
        //     //atMin, move to raise assembly
        // if(excavationState == RAISE_LADDER){
        //     RCLCPP_INFO(this->node->get_logger(), "EXCAVATION AUTONOMY: RAISE_LADDER STATE");
        //     setNeoSpeed(0.0);
        //     excavationState = RAISE_ASSEMBLY;
        // }

        // //Raise assembly
        // if(excavationState == RAISE_ASSEMBLY){
        //     RCLCPP_INFO(this->node->get_logger(), "EXCAVATION AUTONOMY: RAISE_ASSEMBLY STATE");
        //     setShoulderSpeed(-0.8);
            
        //     if(checkErrors(linear1) || checkErrors(linear2)){
        //         excavationState = ERROR_RECOVERY;
        //         errorState = RAISE_ASSEMBLY_ERROR;
        //     }

        //     if(linear1.atMin && linear2.atMin){
        //         setShoulderSpeed(0.0);
        //         excavationState = IDLE;
        //         destination.x=0;
        //         destination.z=0;
        //         setDestDistance(1.0);
        //         setDestAngle(position.yaw + left*180.0);
        //         robotState = GO_TO_HOME;
        //     }
        // }

        // //Handle errors
        // // If Connection error, fail
        // // If potentiometer error, fail
        // if(excavationState == ERROR_RECOVERY){
        //     RCLCPP_INFO(this->node->get_logger(), "EXCAVATION AUTONOMY: IDLE STATE");
        //     if(errorState == LOWER_ASSEMBLY_ERROR || errorState == RAISE_ASSEMBLY_ERROR){
        //         if(linear1.error == "ActuatorNotMovingError" || linear2.error == "ActuatorNotMovingError"){
        //             // Move linear actuators down and up
        //             std_msgs::msg::Float32 speed;
        //             speed.data = -0.8;
        //             shoulderPublisher->publish(speed);
        //             auto start = std::chrono::high_resolution_clock::now();
        //             auto finish = std::chrono::high_resolution_clock::now();
        //             while(std::chrono::duration_cast<std::chrono::nanoseconds>(finish-start).count() < 2000000000){
        //                 finish = std::chrono::high_resolution_clock::now();
        //             }
        //             speed.data = 0.8;
        //             shoulderPublisher->publish(speed);
        //             start = std::chrono::high_resolution_clock::now();
        //             finish = std::chrono::high_resolution_clock::now();
        //             while(std::chrono::duration_cast<std::chrono::nanoseconds>(finish-start).count() < 2000000000){
        //                 finish = std::chrono::high_resolution_clock::now();
        //             }
        //             if(linear1.error == "None" && linear2.error == "None"){
        //                 if(errorState == LOWER_ASSEMBLY_ERROR){
        //                     excavationState = LOWER_ASSEMBLY;
        //                 }
        //                 if(errorState == RAISE_ASSEMBLY_ERROR){
        //                     excavationState = RAISE_ASSEMBLY;
        //                 }
        //                 errorState = NONE;
        //             }
        //             // If it doesn't move, break
        //             else{
        //                 RCLCPP_INFO(this->node->get_logger(), "EXCAVATION AUTONOMY ERROR: AcutatorNotMovingError. Ending Autonomy.");
        //                 excavationState = IDLE;
        //                 robotState = INACTIVE;
        //             }
        //         }
        //         else{
        //             RCLCPP_INFO(this->node->get_logger(), "EXCAVATION AUTONOMY ERROR: PotentiometerError or ConnectionError. Ending Autonomy.");
        //             excavationState = IDLE;
        //             robotState = INACTIVE;
        //         }
        //     }
        // }
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
        RCLCPP_INFO(this->node->get_logger(), "DUMP AUTONOMY: LOWER_ASSEMBLY STATE");
        setShoulderSpeed(0.8);
        
        if(checkErrors(linear1) || checkErrors(linear2)){

        }

        while(!linear1.atMax || !linear2.atMax){}
        setShoulderSpeed(0.0);

        setDumpSpeed(0.8);

        if(checkErrors(linear3)){

        }

        while(!linear3.atMax){}
        setDumpSpeed(-0.8);

        while(!linear3.atMin){}
        setDumpSpeed(0.0);

        RCLCPP_INFO(this->node->get_logger(), "EXCAVATION AUTONOMY: RAISE_ASSEMBLY STATE");
        setShoulderSpeed(-0.8);
        
        if(checkErrors(linear1) || checkErrors(linear2)){

        }

        while(!linear1.atMin || !linear2.atMin){
            
        }
        setShoulderSpeed(0.0);
        robotState = RETURN_TO_START;
    }

    // After dumping the rocks, return to start position and
    // start again
    if(robotState==RETURN_TO_START){

        robotState = ALIGN;
    }
}
    
