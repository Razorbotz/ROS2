#pragma once

#include "Automation.hpp"

class Automation1 : public Automation{

    enum RobotState{LOCATE,GO_TO_DIG_SITE,DIG,HOME,DOCK,DUMP};
    RobotState robotState=GO_TO_DIG_SITE;
    Location destination;

    void automate();

};
