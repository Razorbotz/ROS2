#pragma once

#include "Automation.hpp"


class Automation1 : public Automation{

    enum DumpState{DUMP_IDLE, 
                    DUMP_EXTEND, 
                    DUMP_RETRACT};
    
    DumpState dumpState = DUMP_IDLE;
    
    Location destination;
    float normalDistance = 1.2;

    int destX = 2, destY = 2;
    int stillCounter = 0;
    int counter = 0;

    void automate();

    void publishAutomationOut();

    void setDiagnostics();

    void startAutonomy();

    void setLevel();

    void stopLevel();
};
