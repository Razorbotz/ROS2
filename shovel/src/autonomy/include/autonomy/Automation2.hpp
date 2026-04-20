#pragma once
#include "autonomy/Automation.hpp"

class Automation2 : public Automation {
public:
    Automation2() = default;

    void automate() override;

    void publishAutomationStatus() override;
    void setDiagnostics() override;
    void startAutonomy() override;
    void setLevel() override;
    void stopLevel() override;
    void setDumpMacro() override;
    void setExcavateMacro() override;
    void setExcavate() override;

private:
    void excavateMacro();
    void dumpMacro();
};