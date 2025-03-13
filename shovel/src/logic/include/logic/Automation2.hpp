#pragma once

#include "Automation.hpp"


class Automation2 : public Automation{

    Location destination;
    float normalDistance = 1.2;

    int destX = 2, destY = 2;
    int stillCounter = 0;

    void automate();

    void publishAutomationOut();

    void setDiagnostics();

    void startAutonomy();

    void setLevel();

    void stopLevel();
};
