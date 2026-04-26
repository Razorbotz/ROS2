#include <cstdlib>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <unordered_map>
#include <mutex>

/**
 * @brief Manages cangw rules that bridge a virtual CAN hub (can0) to two
 *        physical USB2CAN interfaces (can1, can2) wired to opposite ends of
 *        the same daisy-chained motor bus.
 *
 * Routing model:
 *   - can0 is the virtual CAN that the rest of the user-space stack uses.
 *   - can1 and can2 are physical adapters, both ALWAYS active on RX so that
 *     status frames from any motor reach can0 regardless of where a break
 *     is in the daisy chain.
 *   - On TX, each motor is routed out exactly ONE physical side -- the side
 *     that can currently reach it. If a break isolates motors A,B,C on the
 *     can1 side and D,E,F on the can2 side, then commands for A,B,C are
 *     filtered onto can1 and commands for D,E,F onto can2. With an intact
 *     bus, every motor is reachable from both sides, so they all default
 *     to can1 (preferred) and we don't double up TX traffic.
 *
 * Whitelist model:
 *   - A motor must (a) be in the authorized motor list AND (b) have a known
 *     reachable side, in order for a TX rule to be installed.
 *   - "Unreachable on both interfaces" -> motor is muted (no TX rule).
 *   - "Unclassified / no status received yet" -> defaults to can1.
 *
 * All cangw mutations re-flush and re-apply the full rule set so there is
 * exactly one source of truth (the cached state below).
 */
class AegisGatewayManager {
public:
    /// Per-motor reachability classification, sourced from status_monitor.
    enum class MotorSide {
        Can1,       ///< Reachable on can1 (also covers "reachable on both")
        Can2,       ///< Reachable only on can2
        Unreachable ///< Not visible on either side -> mute TX
    };

    /**
     * @brief Update both the authorized motor whitelist AND the per-motor
     *        physical routing in a single atomic re-application.
     *
     * @param allowedMotors  Motor IDs (lower 6 bits of extended CAN ID) that
     *                       are authorized to transmit. Empty = mute all TX.
     * @param motorSide      Map from motor ID to which physical side that
     *                       motor lives on. Motors absent from the map fall
     *                       back to Can1 (the preferred default).
     */
    static void setMotorRouting(const std::vector<int>& allowedMotors,
                                const std::unordered_map<int, MotorSide>& motorSide) {
        std::lock_guard<std::mutex> lock(stateMutex());
        lastAllowedMotors() = allowedMotors;
        lastMotorSide() = motorSide;
        allowAllMode() = false;
        applyRulesLocked();
    }

    /**
     * @brief Update only the per-motor routing (e.g. when status_monitor
     *        reports a topology change) without changing the whitelist.
     *        No-op if the side map is identical to the cached one, so it's
     *        cheap to call at the status_monitor publish rate.
     */
    static void updateMotorSides(const std::unordered_map<int, MotorSide>& motorSide) {
        std::lock_guard<std::mutex> lock(stateMutex());
        if (motorSide == lastMotorSide()) {
            return; // No change -> no need to fork+exec cangw.
        }
        lastMotorSide() = motorSide;
        applyRulesLocked();
    }

    /**
     * @brief Update only the authorized whitelist, leaving the cached
     *        per-motor routing in place.
     */
    static void setAllowedMotors(const std::vector<int>& allowedMotors) {
        std::lock_guard<std::mutex> lock(stateMutex());
        lastAllowedMotors() = allowedMotors;
        allowAllMode() = false;
        applyRulesLocked();
    }

    /**
     * @brief Bypass the whitelist and let every outbound frame through. TX
     *        is broadcast to BOTH physical sides so a motor isolated by a
     *        break still receives commands. Used for full-control / debug.
     */
    static void allowAllMotors() {
        std::lock_guard<std::mutex> lock(stateMutex());
        allowAllMode() = true;
        applyRulesLocked();
    }

private:
    // ---- State (function-local statics) ----
    static std::mutex& stateMutex() { static std::mutex m; return m; }
    static std::vector<int>& lastAllowedMotors() { static std::vector<int> v; return v; }
    static std::unordered_map<int, MotorSide>& lastMotorSide() {
        static std::unordered_map<int, MotorSide> m;
        return m;
    }
    static bool& allowAllMode() { static bool b = false; return b; }

    /**
     * @brief Flush all cangw rules and reinstall them from cached state.
     *        Caller must hold stateMutex().
     */
    static void applyRulesLocked() {
        // 1. Flush.
        std::system("cangw -F");

        // 2. RX path: ALWAYS bridge both physical sides into can0, unfiltered.
        //    This guarantees the status monitor and the rest of the stack
        //    see every motor's frames regardless of which side they're on.
        std::system("cangw -A -s can1 -d can0 -e");
        std::system("cangw -A -s can2 -d can0 -e");

        // 3. TX path.
        if (allowAllMode()) {
            // Broadcast to both sides so motors isolated by a break still
            // receive commands.
            std::system("cangw -A -s can0 -d can1 -e");
            std::system("cangw -A -s can0 -d can2 -e");
            std::cout << "[AEGIS] FULL CONTROL GRANTED on can1+can2." << std::endl;
            return;
        }

        if (lastAllowedMotors().empty()) {
            std::cout << "[AEGIS] All motors blocked. Muting transmit." << std::endl;
            return;
        }

        size_t n_can1 = 0, n_can2 = 0, n_muted = 0;
        for (int motorId : lastAllowedMotors()) {
            MotorSide side = MotorSide::Can1; // Default if unclassified
            auto it = lastMotorSide().find(motorId);
            if (it != lastMotorSide().end()) {
                side = it->second;
            }

            if (side == MotorSide::Unreachable) {
                n_muted++;
                continue; // No TX rule installed for unreachable motors.
            }

            const char* phys = (side == MotorSide::Can1) ? "can1" : "can2";
            if (side == MotorSide::Can1) n_can1++; else n_can2++;

            std::stringstream ss;
            // 0x80000000 sets the Extended Frame Flag.
            // 0x8000003F masks to the lowest 6 bits (the motor ID).
            ss << "cangw -A -s can0 -d " << phys << " -e -f 0x800000"
               << std::setfill('0') << std::setw(2) << std::hex << motorId
               << ":0x8000003F";
            std::system(ss.str().c_str());
        }

        std::cout << "[AEGIS] Routing applied. TX: "
                  << n_can1 << " on can1, "
                  << n_can2 << " on can2, "
                  << n_muted << " muted (unreachable)." << std::endl;
    }
};