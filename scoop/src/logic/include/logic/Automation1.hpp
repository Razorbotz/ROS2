#pragma once

#include "Automation.hpp"

class Automation1 : public Automation{

    enum RobotState{LOCATE,ALIGN,GO_TO_DIG_SITE,EXCAVATE,GO_TO_HOME,DOCK,DUMP,RETURN_TO_START,ROBOT_IDLE};
    enum ExcavationState{EXCAVATION_IDLE,LOWER_ASSEMBLY,LOWER_LADDER,DIG,RAISE_LADDER,RAISE_ASSEMBLY,ERROR_RECOVERY};
    enum ErrorState {LOWER_ASSEMBLY_ERROR,LOWER_LADDER_ERROR,DIG_ERROR,RAISE_LADDER_ERROR,RAISE_ASSEMBLY_ERROR,RAISE_BIN_ERROR,LOWER_BIN_ERROR,NONE};
    enum DumpState{DUMP_IDLE,DUMP_LOWER_ASSEMBLY,RAISE_BIN,SERVO,LOWER_BIN,DUMP_RAISE_ASSEMBLY,DUMP_ERROR_RECOVERY};
    RobotState robotState=LOCATE;
    ExcavationState excavationState = EXCAVATION_IDLE;
    ErrorState errorState = NONE;
    DumpState dumpState = DUMP_IDLE;
    Location destination;

    void automate();

};
