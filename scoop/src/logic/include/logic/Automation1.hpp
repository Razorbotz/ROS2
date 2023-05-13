#pragma once

#include "Automation.hpp"


class Automation1 : public Automation{

    enum RobotState{LOCATE,ALIGN,GO_TO_DIG_SITE,EXCAVATE,GO_TO_HOME,DOCK,DUMP,RETURN_TO_START,ROBOT_IDLE};
    enum ExcavationState{EXCAVATION_IDLE,LOWER_ASSEMBLY,LOWER_LADDER,DIG,RAISE_LADDER,RAISE_ASSEMBLY,ERROR_RECOVERY};
    enum ErrorState {LOWER_ASSEMBLY_ERROR,LOWER_LADDER_ERROR,DIG_ERROR,RAISE_LADDER_ERROR,RAISE_ASSEMBLY_ERROR,RAISE_BIN_ERROR,LOWER_BIN_ERROR,NONE};
    enum DumpState{DUMP_IDLE,DUMP_LOWER_ASSEMBLY,RAISE_BIN,SERVO,LOWER_BIN,DUMP_RAISE_ASSEMBLY,DUMP_ERROR_RECOVERY};
    RobotState robotState=EXCAVATE;
    ExcavationState excavationState = EXCAVATION_IDLE;
    ErrorState errorState = NONE;
    DumpState dumpState = DUMP_IDLE;
    Location destination;

    std::map<RobotState, const char*> robotStateMap = {
        {LOCATE, "Locate"},
        {ALIGN, "Align"},
        {GO_TO_DIG_SITE, "Go To Dig Site"},
        {EXCAVATE, "Excavate"},
        {GO_TO_HOME, "Go To Home"},
        {DOCK, "Dock"},
        {DUMP, "Dump"},
        {RETURN_TO_START, "Return To Start"},
        {ROBOT_IDLE,  "Idle"}
    };

    std::map<ExcavationState, const char*> excavationStateMap = {
        {EXCAVATION_IDLE, "Idle"},
        {LOWER_ASSEMBLY, "Lower Assembly"},
        {LOWER_LADDER, "Lower Ladder"},
        {DIG, "Dig"},
        {RAISE_LADDER, "Raise Ladder"},
        {RAISE_ASSEMBLY, "Raise Assembly"},
        {ERROR_RECOVERY, "Error Recovery"}
    };

    std::map<ErrorState, const char*> errorStateMap = {
        {LOWER_ASSEMBLY_ERROR, "Lower Assembly Error"},
        {LOWER_LADDER_ERROR, "Lower Ladder Error"},
        {DIG_ERROR, "Dig Error"},
        {RAISE_LADDER_ERROR, "Raise Ladder Error"},
        {RAISE_ASSEMBLY_ERROR, "Raise Assembly Error"},
        {RAISE_BIN_ERROR, "Raise Bin Error"},
        {LOWER_BIN_ERROR, "Lower Bin Error"},
        {NONE, "None"}
    };

    std::map<DumpState, const char*> dumpStateMap = {
        {DUMP_IDLE, "Idle"},
        {DUMP_LOWER_ASSEMBLY, "Lower Assembly"},
        {RAISE_BIN, "Raise Bin"},
        {SERVO, "Servo"},
        {LOWER_BIN, "Lower Bin"},
        {DUMP_RAISE_ASSEMBLY, "Raise Assembly"},
        {DUMP_ERROR_RECOVERY, "Error Recovery"}
    };

    void automate();

    void publishAutomationOut();

};
