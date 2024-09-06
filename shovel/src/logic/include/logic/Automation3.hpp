#pragma once

#include "Automation.hpp"


class Automation3 : public Automation{

    enum RobotState{INITIAL,DIAGNOSTICS,LOCATE,ALIGN,GO_TO_DIG_SITE,EXCAVATE,OBSTACLE,GO_TO_HOME,DOCK,DUMP,RETURN_TO_START,ROBOT_IDLE};
    enum ExcavationState{EXCAVATION_IDLE,SQUARE_UP,RAISE_ARM,RAISE_BUCKET,COLLECT,LOWER_ARM,LOWER_BUCKET,EXCAVATION_ERROR_RECOVERY};
    enum ErrorState {TALON_14_ERROR, TALON_15_ERROR, TALON_16_ERROR, TALON_17_ERROR, FALCON_10_ERROR, FALCON_11_ERROR, FALCON_12_ERROR, FALCON_13_ERROR,NONE};
    enum DiagnosticsState{DIAGNOSTICS_IDLE,TALON_EXTEND,TALON_RETRACT,FALCON_FORWARD,DIAGNOSTICS_ERROR_RECOVERY};
    RobotState robotState = INITIAL;
    RobotState previousState = ROBOT_IDLE;
    ExcavationState excavationState = EXCAVATION_IDLE;
    ErrorState errorState = NONE;
    DiagnosticsState diagnosticsState = TALON_EXTEND;
    Location destination;
    float normalDistance = 1.2;

    float destX = 1, destY = 3;
    int stillCounter = 0;

    std::map<RobotState, const char*> robotStateMap = {
        {DIAGNOSTICS, "Diagnostics"},
        {LOCATE, "Locate"},
        {ALIGN, "Align"},
        {GO_TO_DIG_SITE, "Go To Dig Site"},
        {EXCAVATE, "Excavate"},
        {OBSTACLE, "Obstacle"},
        {GO_TO_HOME, "Go To Home"},
        {DOCK, "Dock"},
        {DUMP, "Dump"},
        {RETURN_TO_START, "Return To Start"},
        {ROBOT_IDLE,  "Idle"}
    };

    std::map<ExcavationState, const char*> excavationStateMap = {
        {EXCAVATION_IDLE, "Idle"},
        {RAISE_ARM, "Raise Arm"},
        {RAISE_BUCKET, "Raise Bucket"},
        {COLLECT, "Collect"},
        {LOWER_ARM, "Lower Arm"},
        {LOWER_BUCKET, "Lower Bucket"},
        {EXCAVATION_ERROR_RECOVERY, "Error Recovery"}
    };

    std::map<ErrorState, const char*> errorStateMap = {
        {TALON_14_ERROR, "Talon 14 Error"},
        {TALON_15_ERROR, "Talon 15 Error"},
        {TALON_16_ERROR, "Talon 16 Error"},
        {TALON_17_ERROR, "Talon 17 Error"},
        {FALCON_10_ERROR, "Falcon 10 Error"},
        {FALCON_11_ERROR, "Falcon 11 Error"},
        {FALCON_12_ERROR, "Falcon 12 Error"},
        {FALCON_13_ERROR, "Falcon 13 Error"},
        {NONE, "None"}
    };

    std::map<DiagnosticsState, const char*> diagnosticsStateMap = {
        {DIAGNOSTICS_IDLE, "Idle"},
        {TALON_EXTEND, "Talon Extend"},
        {TALON_RETRACT, "Talon Retract"},
        {FALCON_FORWARD, "Falcon Forward"},
        {DIAGNOSTICS_ERROR_RECOVERY, "Error Recovery"}
    };

    void automate();

    void publishAutomationOut();

    void setDiagnostics();

    void startAutonomy();
};
