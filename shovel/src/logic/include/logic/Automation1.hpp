#pragma once

#include "Automation.hpp"


class Automation1 : public Automation{   
    Location destination;
    float normalDistance = 1.2;
    bool inExcavation = false;

    float currentX = 0.0, currentZ = 0;
    int stillCounter = 0;
    int counter = 0;
    bool dump = true;
    bool excavate = true;

    void automate();

    void publishAutomationOut();

    void setDiagnostics();

    void startAutonomy();

    void setLevel();

    void stopLevel();

    void setDump();

    void dumpMacro();
    
    void setExcavate();

    void excavateMacro();
};
