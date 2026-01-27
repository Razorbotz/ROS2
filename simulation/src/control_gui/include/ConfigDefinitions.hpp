#pragma once

#include <string>
#include <vector>
#include <map>
#include <set>
#include <gtkmm.h>
#include "InfoFrame.hpp"
#include "BinaryMessage.hpp"

// --- Element Type Definitions ---
enum class ElementType {
    UInt8, UInt16, Int8, Int32, Float32, Boolean, String
};

struct ElementInfo {
    ElementType type;
    std::string name;
};

// --- Global UI State Variables ---
extern std::string lightBackgroundColor;
extern std::string darkBackgroundColor;
extern bool displaySpeed;
extern bool numbersInside;
extern bool numberTicks;
extern bool allowConfig;

// --- Accessor Functions for all Data ---

std::string& get_configFile();

// --- Speedometer ---
std::set<std::string>& get_speedometer_keys();

// --- Subsystem Data ---
std::vector<std::string>& get_talon_keys();
std::vector<std::string>& get_reset_talon_keys();
std::map<std::string, bool>& get_talon_values();

std::vector<std::string>& get_falcon_keys();
std::vector<std::string>& get_reset_falcon_keys();
std::map<std::string, bool>& get_falcon_values();

std::vector<std::string>& get_linear_keys();
std::vector<std::string>& get_reset_linear_keys();
std::map<std::string, bool>& get_linear_values();

std::vector<std::string>& get_power_keys();
std::vector<std::string>& get_reset_power_keys();
std::map<std::string, bool>& get_power_values();

std::vector<std::string>& get_power2_keys();
std::vector<std::string>& get_reset_power2_keys();
std::map<std::string, bool>& get_power2_values();

std::vector<std::string>& get_autonomy_keys();
std::vector<std::string>& get_reset_autonomy_keys();
std::map<std::string, bool>& get_autonomy_values();

std::vector<std::string>& get_zed_keys();
std::vector<std::string>& get_reset_zed_keys();
std::map<std::string, bool>& get_zed_values();

std::vector<std::string>& get_communication_keys();
std::vector<std::string>& get_reset_communication_keys();
std::map<std::string, bool>& get_communication_values();

std::vector<std::string>& get_drivetrain_keys();
std::vector<std::string>& get_reset_drivetrain_keys();
std::map<std::string, bool>& get_drivetrain_values();

// --- Central Maps ---
std::map<std::string, std::vector<std::string>*>& get_key_vectors();
std::map<std::string, std::vector<ElementInfo>>& get_element_definitions();

// --- Helper Functions ---
void initialize_maps();
std::vector<std::string> getKeys(const std::string& label);
std::map<std::string, bool>& getMap(std::string label);
std::string getNameFromPrefix(std::string label);

// --- UI Helper Functions (Moved from control.cpp) ---
Gdk::RGBA parse_color(const std::string& color_str);
std::string to_color_string(const Gdk::RGBA& color);
void addElementToInfoFrame(InfoFrame* frame, const Element& element);
void updateGUI();