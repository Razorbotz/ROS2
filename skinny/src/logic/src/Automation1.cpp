
#include <cmath>

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
    }else if (deltaYaw < yawTolerance){
        changeSpeed(0.15,-0.15);
    }else{
         changeSpeed(0.15 - 0.1*deltaYaw/yawTolerance,0.15 + 0.1*deltaYaw/yawTolerance);
    }
    std::cout << orientation.roll*180/M_PI << ", " << orientation.pitch*180/M_PI << ", " << orientation.yaw*180/ M_PI << "   "
              << "   \t" << position.x << "  " << position.y << "  " << position.z
              << "   \t" << position.ox << "  " << position.oy << "  " << position.oz << "  " << position.ow
              << "   \t" << facingUnitX << " " << facingUnitZ << "   " << yaw << " " << deltaYaw << " " << theta
              << "   \t" << position.arucoVisible << std::endl;
    }else{
        changeSpeed(0,0);
    }
}
    
