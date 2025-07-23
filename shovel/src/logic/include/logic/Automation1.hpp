#pragma once

#include "Automation.hpp"


class Automation1 : public Automation{   
    Location destination;
    float normalDistance = 1.2;
    bool inExcavation = false;

    int stillCounter = 0;
    int counter = 0;

    void automate();

    void publishAutomationStatus();

    void setDiagnostics();

    void startAutonomy();

    void setLevel();

    void stopLevel();

    void setDump();

    void dumpMacro();
    
    void setExcavate();

    void excavateMacro();
};
