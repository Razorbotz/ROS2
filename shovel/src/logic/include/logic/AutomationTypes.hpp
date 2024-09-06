#pragma once

struct Position{
    double x,y,z,ox,oy,oz,ow,arucoVisible,roll,pitch,yaw,aruco_roll,aruco_pitch,aruco_yaw,x_acc, y_acc, z_acc, x_vel, y_vel, z_vel, arucoInitialized;
};

struct Location{
    double x,z;
};

struct Quaternion {
    double w, x, y, z;
};

struct EulerAngles {
    double roll, pitch, yaw;
};

struct Linear {
    float speed = 0.0;
    std::string error = "None";
    bool atMin = false;
    bool atMax = false;
    float distance = 0.0;
    float stroke = 0.0;
    float extensionSpeed = 0.0;
    float timeToExtend = 0.0;
    int potentiometer = 0;
    bool sensorless = false;
    float maxCurrent = 0.0;
};

struct MotorOut{
    float busVoltage;
    float outputCurrent;
    float outputVoltage;
    float outputPercentage;
    float maxCurrent = 0.0;
};

struct Arena{
    int height;
    int width;
    int excavationArea;
    int obstacleArea;
    int targetArea [4];
};