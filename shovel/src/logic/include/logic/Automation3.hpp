#pragma once

#include "Automation.hpp"


class Automation3 : public Automation{
    
    Location destination;
    float normalDistance = 1.2;

    float destX = 1, destY = 3;
    int stillCounter = 0;

    void automate();

    void publishAutomationOut();

    void setDiagnostics();

    void startAutonomy();

    void setLevel();
};
