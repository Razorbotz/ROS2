#pragma once

#include "Automation.hpp"

class Automation1 : public Automation{

    enum RobotState{LOCATE,ALIGN,GO_TO_DIG_SITE,EXCAVATE,GO_TO_HOME,DOCK,DUMP,RETURN_TO_START,INACTIVE};
    enum ExcavationState{IDLE,LOWER_ASSEMBLY,LOWER_LADDER,DIG,RAISE_LADDER,RAISE_ASSEMBLY,ERROR_RECOVERY};
    enum ErrorState {LOWER_ASSEMBLY_ERROR,LOWER_LADDER_ERROR,DIG_ERROR,RAISE_LADDER_ERROR,RAISE_ASSEMBLY_ERROR,NONE};
    RobotState robotState=EXCAVATE;
    ExcavationState excavationState = IDLE;
    ErrorState errorState = NONE;
    Location destination;

    void automate();

};
