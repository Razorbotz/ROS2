#pragma once

/** @file
 *
 *  @brief Header file for AutomationTypes
 * 
 * */

struct Position{
    double x,y,z,ox,oy,oz,ow,arucoVisible;
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
