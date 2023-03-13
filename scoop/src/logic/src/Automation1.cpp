
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

    // After reaching teh excavation area, go through mining
    // sequence
    if(robotState==DIG){
        //Lower assembly
            //Set linear actuators to 0.4
            //Check for errors
            //Handle errors
            //atMax, move to lower ladder

        //Lower ladder
            //Set speed to 0.4
            //Check for errors
            //Handle errors
            //atMax, move to dig

        //Dig
        
        //Raise ladder
            //Set speed to -0.4
            //Check for errors
            //Handle errors
            //atMin, move to raise assembly

        //Raise assembly
            //Set speed to -0.4
            //Check for errors
            //Handle errors
            //atMin, move to Idle

        //robotState = GO_TO_HOME;
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
    
