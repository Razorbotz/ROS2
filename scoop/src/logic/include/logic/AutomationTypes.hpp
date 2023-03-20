#pragma once

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

struct Linear {
    float speed = 0.0;
    std::string error = "ConnectionError";
    bool atMin = false;
    bool atMax = false;
};