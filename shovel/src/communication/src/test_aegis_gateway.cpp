#include <cstdlib>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <iostream>

// Toggle this to 'true' when compiling on the Jetsons, 
// and 'false' when testing locally on WSL.
const bool EXECUTE_SYSTEM_CALLS = false; 

class AegisGatewayManager {
private:
    // Helper function to either execute or print the command
    static void executeCommand(const std::string& cmd) {
        if (EXECUTE_SYSTEM_CALLS) {
            std::system(cmd.c_str());
        }
        else {
            std::cout << "  [WSL DRY RUN] -> " << cmd << std::endl;
        }
    }

public:
    static void setAllowedMotors(const std::vector<int>& allowedMotors) {
        executeCommand("sudo cangw -F");
        executeCommand("sudo cangw -A -s can2 -d can0 -e");

        if (allowedMotors.empty()) {
            std::cout << "[AEGIS] All motors blocked. Muting transmit.\n" << std::endl;
            return;
        }

        for (int motorId : allowedMotors) {
            std::stringstream ss;
            ss << "sudo cangw -A -s can0 -d can2 -e -f 0x800000" 
               << std::setfill('0') << std::setw(2) << std::hex << motorId 
               << ":0x8000003F";
            executeCommand(ss.str());
        }

        std::cout << "[AEGIS] Routing applied. Authorized to control " 
                  << allowedMotors.size() << " motor(s).\n" << std::endl;
    }
    
    static void allowAllMotors() {
        executeCommand("sudo cangw -F");
        executeCommand("sudo cangw -A -s can2 -d can0 -e");
        executeCommand("sudo cangw -A -s can0 -d can2 -e");
        std::cout << "[AEGIS] FULL CONTROL GRANTED. All motors allowed.\n" << std::endl;
    }
};

// --- Interactive Test Menu ---
int main() {
    int choice = 0;

    std::cout << "=== AEGIS Gateway Test Environment ===\n";

    while (true) {
        std::cout << "Select a state to apply:\n";
        std::cout << "1. STANDBY (Mute all transmit)\n";
        std::cout << "2. SPLIT: Orin Control (Motors 1-14)\n";
        std::cout << "3. SPLIT: Nano Control (Motor 15 only)\n";
        std::cout << "4. PRIMARY: Full Chassis Control\n";
        std::cout << "5. Exit\n";
        std::cout << "Enter choice: ";
        std::cin >> choice;

        std::vector<int> motors;

        switch (choice) {
            case 1:
                AegisGatewayManager::setAllowedMotors({});
                break;
            case 2:
                motors = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
                AegisGatewayManager::setAllowedMotors(motors);
                break;
            case 3:
                motors = {15};
                AegisGatewayManager::setAllowedMotors(motors);
                break;
            case 4:
                AegisGatewayManager::allowAllMotors();
                break;
            case 5:
                std::cout << "Exiting test...\n";
                return 0;
            default:
                std::cout << "Invalid choice.\n";
                continue;
        }

        // Immediately print the active kernel rules so you can verify the bitmasks
        std::cout << "--- ACTIVE CANGW RULES ---\n";
        std::system("cangw -L");
        std::cout << "--------------------------\n\n";
    }

    return 0;
}