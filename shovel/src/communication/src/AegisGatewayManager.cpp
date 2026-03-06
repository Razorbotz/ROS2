#include <cstdlib>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <iostream>

class AegisGatewayManager {
public:
    /**
     * @brief Dynamically sets which motors are allowed to transmit from the virtual bus to the physical wire.
     * @param allowedMotors Vector of motor IDs to whitelist. If empty, transmit is muted.
     * @param isOrin Set to true if running on the Orin to apply the NVIDIA loopback protection fix.
     */
    static void setAllowedMotors(const std::vector<int>& allowedMotors) {
        // 1. Flush all existing rules to prevent duplicates
        std::system("sudo cangw -F");

        // 2. Re-establish the Receive Path (Physical can2 -> Virtual can0)
        // The Orin requires the '-x' flag due to the mttcan loopback bug. The Nano does not.
        std::system("sudo cangw -A -s can2 -d can0 -e");

        // 3. Handle total mute (STANDBY state)
        if (allowedMotors.empty()) {
            std::cout << "[AEGIS] All motors blocked. Muting transmit." << std::endl;
            return;
        }

        // 4. Apply the Whitelist Transmit Rules (Virtual can0 -> Physical can2)
        for (int motorId : allowedMotors) {
            std::stringstream ss;
            
            // 0x80000000 is the Extended Frame Flag.
            // 0x8000003F is the mask to isolate the lowest 6 bits (the motor ID).
            ss << "sudo cangw -A -s can0 -d can2 -e -f 0x800000" 
               << std::setfill('0') << std::setw(2) << std::hex << motorId 
               << ":0x8000003F";
            
            std::system(ss.str().c_str());
        }

        std::cout << "[AEGIS] Routing applied. Authorized to control " 
                  << allowedMotors.size() << " motor(s)." << std::endl;
    }
    
    /**
     * @brief Helper function to instantly grant control over the entire chassis.
     */
    static void allowAllMotors() {
        std::system("sudo cangw -F");
        std::system("sudo cangw -A -s can2 -d can0 -e");
        
        // Blindly route all outbound traffic
        std::system("sudo cangw -A -s can0 -d can2 -e");
        std::cout << "[AEGIS] FULL CONTROL GRANTED. All motors allowed." << std::endl;
    }
};