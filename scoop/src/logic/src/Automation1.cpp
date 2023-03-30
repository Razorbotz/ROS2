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
    // Initially start with locating the Aruco marker
    // Turn slowly until it's seen
    if(robotState==LOCATE){
        changeSpeed(0.15,-0.15);
        if(position.arucoVisible==true){
            robotState=GO_TO_DIG_SITE;
            destination.x=-5;
            destination.z=2;
            changeSpeed(0,0);
        }
    }
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
    else{
        changeSpeed(0,0);
    }
    

    // After finding the Aruco marker, turn the bot to 
    // align with the arena
    if(robotState==ALIGN){

        //robotState = GO_TO_DIG_SITE;
    }

    // After aligning with the arena, navigate to the 
    // excavation area
    if(robotState==GO_TO_DIG_SITE){
        
        //robotState = DIG;
    }

    // After reaching the excavation area, go through mining
    // sequence
    if(robotState==EXCAVATE){
        if(excavationState == IDLE){
            RCLCPP_INFO(this->node->get_logger(), "EXCAVATION AUTONOMY: IDLE STATE");
            excavationState = LOWER_ASSEMBLY;
        }
        if(excavationState == LOWER_ASSEMBLY){
            RCLCPP_INFO(this->node->get_logger(), "EXCAVATION AUTONOMY: LOWER_ASSEMBLY STATE");
            setShoulderSpeed(0.8);
            
            if(checkErrors(linear1) || checkErrors(linear2)){
                excavationState = ERROR_RECOVERY;
                errorState = LOWER_ASSEMBLY_ERROR;
            }

            if(linear1.atMax && linear2.atMax){
                setShoulderSpeed(0.0);
                excavationState = LOWER_LADDER;
            }
        }

        //Lower ladder
            //Set speed to 0.4
            
            //Check for errors
            
            //Handle errors
            
            //atMax, move to dig
        if(excavationState == LOWER_LADDER){
            RCLCPP_INFO(this->node->get_logger(), "EXCAVATION AUTONOMY: LOWER_LADDER STATE");
            setNeoSpeed(0.1);
            excavationState = DIG;
        }

        //Dig
        // Have ladder fully extended and spinning
        // Back up robot to get more regolith
        if(excavationState == DIG){
            RCLCPP_INFO(this->node->get_logger(), "EXCAVATION AUTONOMY: DIG STATE");
            excavationState = RAISE_LADDER;
        }
        
        //Raise ladder
            //Set speed to -0.4
            
            //Check for errors
            
            //Handle errors
            
            //atMin, move to raise assembly
        if(excavationState == RAISE_LADDER){
            RCLCPP_INFO(this->node->get_logger(), "EXCAVATION AUTONOMY: RAISE_LADDER STATE");
            setNeoSpeed(0.0);
            excavationState = RAISE_ASSEMBLY;
        }

        //Raise assembly
        if(excavationState == RAISE_ASSEMBLY){
            RCLCPP_INFO(this->node->get_logger(), "EXCAVATION AUTONOMY: RAISE_ASSEMBLY STATE");
            setShoulderSpeed(-0.8);
            
            if(checkErrors(linear1) || checkErrors(linear2)){
                excavationState = ERROR_RECOVERY;
                errorState = RAISE_ASSEMBLY_ERROR;
            }

            if(linear1.atMin && linear2.atMin){
                setShoulderSpeed(0.0);
                excavationState = IDLE;
                robotState = GO_TO_HOME;
            }
        }

        //Handle errors
        // If Connection error, fail
        // If potentiometer error, fail
        if(excavationState == ERROR_RECOVERY){
            RCLCPP_INFO(this->node->get_logger(), "EXCAVATION AUTONOMY: IDLE STATE");
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
                        excavationState = IDLE;
                        robotState = INACTIVE;
                    }
                }
                else{
                    RCLCPP_INFO(this->node->get_logger(), "EXCAVATION AUTONOMY ERROR: PotentiometerError or ConnectionError. Ending Autonomy.");
                    excavationState = IDLE;
                    robotState = INACTIVE;
                }
            }
        }
    }

    // After mining, return to start position
    if(robotState==GO_TO_HOME){

        //robotState = DOCK;
    }

    // After reaching start position, dock at dump bin
    if(robotState==DOCK){

        //robotState = DUMP;
    }

    // Dump the collected rocks in the dump bin
    if(robotState==DUMP){
        
        //robotState = RETURN_TO_START;
    }

    // After dumping the rocks, return to start position and
    // start again
    if(robotState==RETURN_TO_START){

        //robotState = ALIGN;
    }
    else{
        changeSpeed(0,0);
    }
}
    
