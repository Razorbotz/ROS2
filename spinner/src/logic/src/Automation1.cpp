
#include <cmath>
#include <ctime>

#include "logic/Automation.hpp"
#include "logic/Automation1.hpp"


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
    }
    if(robotState==DIG){
        //Move arm to starting location
        changeArmSpeed(0.0);
        changeShoulderSpeed(0.0);
        
        //Lower arm near ground
        changeArmSpeed(0.0);
        changeShoulderSpeed(0.0);

        //Start drum
        changeDrumSpeed(1.0);

        //Spin for some amount of time
        time_t startTime = time(NULL);
        time_t currentTime = time(NULL);
        while((currentTime - startTime) < 15)
            currentTime = time(NULL);

        //Lower arm further
        changeArmSpeed(0.0);
        changeShoulderSpeed(0.0);

        //Raise arm
        changeArmSpeed(0.0);
        changeShoulderSpeed(0.0);

        //Stop drum
        changeDrumSpeed(0.0);

        //Raise arm and rotate it around to dump
        changeArmSpeed(0.0);
        changeShoulderSpeed(0.0);

        //Spin drum backwards to unload
        changeDrumSpeed(-1.0);

        //Spin for some amount of time
        startTime = time(NULL);
        currentTime = time(NULL);
        while((currentTime - startTime) < 15)
            currentTime = time(NULL);

        //Stop drum
        changeDrumSpeed(0.0);

        //Rotate arm back around and lower to starting location
        changeArmSpeed(0.0);
        changeShoulderSpeed(0.0);

    }
    if(robotState==DUMP){
        //Extend the dump bin fully
        //Given linear actuator length of 12", 11.8" usable
        //and a worst-case, fully loaded speed of .48"/s, 
        //then the max runtime is ~25 seconds.  Without any
        //feedback from potentiometers, assume worst-case
        //and run from there.  Limit switches will prevent
        //damage from extending too far.
        time_t startTime = time(NULL);
        changeDumpBinSpeed(1.0);
        time_t currentTime = time(NULL);
        while((currentTime - startTime) < 25)
            currentTime = time(NULL);
        changeDumpBinSpeed(0.0);

        //Open lock servo
        changeLockServoSpeed(0.0);

        //Open arm servo
        changeArmServoSpeed(0.0);

        //Close lock servo
        changeLockServoSpeed(0.0);

        //Close lock servo
        changeArmServoSpeed(0.0);

        //Lower dump bin
        startTime = time(NULL);
        changeDumpBinSpeed(-1.0);
        currentTime = time(NULL);
        while((currentTime - startTime) < 25)
            currentTime = time(NULL);
        changeDumpBinSpeed(0.0);
    }
    else{
        changeSpeed(0,0);
    }
}
    
