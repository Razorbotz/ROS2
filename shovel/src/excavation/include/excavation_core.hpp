#ifndef EXCAVATION_CORE_HPP
#define EXCAVATION_CORE_HPP

#include <cmath>
#include <string>
#include <algorithm> // for std::max/min if needed

constexpr int POT_FLOAT_LOW = 100;
constexpr int POT_FLOAT_HIGH = 110;
constexpr int POT_RAW_MIN_VALID = 30;
constexpr int POT_RAW_MAX_VALID = 980;
constexpr int POT_RAW_RANGE = POT_RAW_MAX_VALID - POT_RAW_MIN_VALID;
constexpr int NOISE_THRESH = 2;
constexpr int NO_MOVEMENT_LIMIT = 15;

// Distance sync thresholds (inches)
constexpr float distThresh1 = 0.05f;
constexpr float distThresh2 = 0.10f;
constexpr float distThresh3 = 0.15f;

enum Error {
    ActuatorsSyncError,
    ActuatorNotMovingError,
    PotentiometerError,
    None
};

inline const char* getErrorString(Error e) {
    switch(e) {
        case ActuatorsSyncError: return "ActuatorsSyncError";
        case ActuatorNotMovingError: return "ActuatorNotMovingError";
        case PotentiometerError: return "PotentiometerError";
        default: return "None";
    }
}

namespace core{
    struct LinearActuator{
        int motorNumber = 0;
        float speed = 0.0;              // Speed variable of linear actuator
        int potentiometer = 0;          // Potentiometer reading
        int timeWithoutChange = 0;      // Number of potentiometer values received without change when speed > 0
        int max = 0;                    // Max potentiometer value
        int min = 1024;                 // Min potentiometer value
        Error error = None;             // Error state of the actuator
        bool atMin = false;             // Bool value of if actuator is at min extension
        bool atMax = false;             // Bool value of if actuator is at max extension
        float stroke = 11.8;            // Length of stroke of the actuator
        float distance = 0.0;           // Distance extended
        float extensionSpeed = 0.0;     // Speed of extension in in/sec
        float timeToExtend = 0.0;       // Time to fully extend actuator
        bool sensorless = false;        // Running without sensor
        float maxCurrent = 0.0;         
        bool initialized = false;
        float previousSpeed = 0.0;
        int previousPotent = 0;
        //float lowerDistance = 0.0;
        //float upperDistance = 0.0;
        //float lowerSpeed = 0.0;
        //float upperSpeed = 0.0;
        LinearActuator(int motor, float strokeLength, float ExtensionSpeed, float TimeToExtend)
            : motorNumber(motor), stroke(strokeLength), extensionSpeed(ExtensionSpeed), timeToExtend(TimeToExtend) {}
    };

    // --- Pure Logic Functions ---
    // Checks if potentiometer value is within floating range
    inline bool isFloatValue(int v) {
        return v >= POT_FLOAT_LOW && v <= POT_FLOAT_HIGH;
    }

    // Checks that potentiometer value is within valid range
    inline bool isRealValue(int v) {
        return v >= POT_RAW_MIN_VALID && v <= POT_RAW_MAX_VALID;
    }

    // --- Helper Logic ---
    inline void setSpeedAtEnd(LinearActuator* a, float currentSpeed) {
        if ((a->atMax && currentSpeed > 0) || (a->atMin && currentSpeed < 0))
            a->speed = 0.0f;
    }

    // --- Core Logic Functions ---
    /** @brief Function to sync the linear actuators. 
     * 
     * The sync function works by checking if the currentSpeed is
     * greater than zero. If the speed is greater than zero, the val
     * checks which actuator is more extended and sets the speed of
     * the actuator to a lower value if the diff is greater than the 
     * thresh values.  If the value is less than zero, the val checks 
     * which actuator is less extended and sets the speed of the 
     * actuator to a lower value.
     * @return void
     * */
    /*
        val truth table:
        if Current Speed > 0:                   If actuators are extending
            if linear1.pot >= linear2.pot:      If linear1 is further extended, use first value in ternary operators below
                val = true
            else:
                val = false
        else:                                   If actuators are retracting
            if linear1.pot < linear2.pot:       If linear1 is further retracted, use first value in ternary operators below
                val = true
            else:
                val = false
        */
    inline void sync(LinearActuator* a, LinearActuator* b, float currentSpeed) {
        float diff = std::abs(a->potentiometer - b->potentiometer);
        bool aIsAhead = (currentSpeed > 0)
                        ? (a->potentiometer >= b->potentiometer)
                        : (a->potentiometer <  b->potentiometer);

        float scale = 950.0f / a->stroke;

        if (diff > scale / 6.0f) {
            if (aIsAhead) a->speed = 0.0f;
            else          b->speed = 0.0f;
        }
        else if (diff > scale / 9.0f) {
            if (aIsAhead) a->speed *= 0.5f;
            else          b->speed *= 0.5f;
        }
        else if (diff > scale / 12.0f) {
            if (aIsAhead) a->speed *= 0.9f;
            else          b->speed *= 0.9f;
        }
        else {
            a->speed = currentSpeed;
            b->speed = currentSpeed;
        }
    }

    /** @brief Function to sync the linear actuators when using the distance 
     * calculated from the time running. 
     * 
     * The sync function works by checking if the currentSpeed is
     * greater than zero. If the speed is greater than zero, the val
     * checks which actuator is more extended and sets the speed of
     * the actuator to a lower value if the diff is greater than the 
     * thresh values.  If the value is less than zero, the val checks 
     * which actuator is less extended and sets the speed of the 
     * actuator to a lower value. 
     * @return void
     * */
    inline void syncDistance(LinearActuator* a, LinearActuator* b, float currentSpeed) {
        float diff = std::abs(a->distance - b->distance);
        bool aIsAhead = (currentSpeed > 0)
                        ? (a->distance >= b->distance)
                        : (a->distance <  b->distance);

        if (diff > distThresh3) {
            if (aIsAhead) a->speed = 0.0f;
            else          b->speed = 0.0f;

            if (!a->sensorless) a->error = ActuatorsSyncError;
            if (!b->sensorless) b->error = ActuatorsSyncError;
        }
        else if (diff > distThresh2) {
            if (aIsAhead) a->speed *= 0.5f;
            else          b->speed *= 0.5f;

            if (!a->sensorless && a->error == ActuatorsSyncError) a->error = None;
            if (!b->sensorless && b->error == ActuatorsSyncError) b->error = None;
        }
        else if (diff > distThresh1) {
            if (aIsAhead) a->speed *= 0.9f;
            else          b->speed *= 0.9f;

            if (!a->sensorless && a->error == ActuatorsSyncError) a->error = None;
            if (!b->sensorless && b->error == ActuatorsSyncError) b->error = None;
        }
        else {
            a->speed = currentSpeed;
            b->speed = currentSpeed;
            if (!a->sensorless && a->error == ActuatorsSyncError) a->error = None;
            if (!b->sensorless && b->error == ActuatorsSyncError) b->error = None;
        }
    }

    /** @brief Function to process potentiometer data. 
     * 
     * This function processes the passed potentiometer data
     * and adjusts the passed linear values accordingly. First
     * the function sets the min and max values if the new data
     * is beyond the previous limits. Next, the function checks
     * if the value is within a threshold of the previous value
     * that is stored in the linear->potentiometer variable. If
     * the value is within this threshold, it's assumed that 
     * the actuator isn't moving. If the speed isn't equal to
     * zero, ie the actuator should be moving, the timeWithoutChange
     * variable gets increased. If the timeWithoutChange is greater than 5,
     * the function checks if the actuator is at the min or max
     * positions and sets the corresponding values to true if
     * it is.  If the data is outside of the threshold, the 
     * actuator is moving as intended and is not at the min or
     * max positions.
     * 
     * NOTE: If the potentiometer is disconnected, the values fall to
     * between 100 and 110.
     * @param potentData - Int value of potentiometer
     * @param *linear - Pointer to linear object
     * @return bool - errorLogged
     * */
    inline bool processPotentiometer(int potentData, LinearActuator* linear, bool runSystem) {
        bool errorLogged = false;

        // Track observed min/max
        if (potentData < linear->min) linear->min = potentData;
        if (potentData > linear->max) linear->max = potentData;

        // Initialization
        if (!linear->initialized) {
            if (!isFloatValue(potentData) && isRealValue(potentData)) {
                linear->initialized = true;
            }
        }

        // Disconnect check
        if (linear->initialized && isFloatValue(potentData)) {
            if (std::abs(linear->potentiometer - potentData) > 50) {
                linear->sensorless = true;
                linear->error = PotentiometerError;
            }
        }

        if (isRealValue(potentData)) {
            linear->distance = linear->stroke * (static_cast<float>(potentData - POT_RAW_MIN_VALID) / POT_RAW_RANGE);
        }

        // Not-moving detection
        if (linear->potentiometer >= potentData - NOISE_THRESH && linear->potentiometer <= potentData + NOISE_THRESH) {
            if (linear->speed != 0.0f && runSystem) {
                linear->timeWithoutChange += 1;
                if (linear->timeWithoutChange >= NO_MOVEMENT_LIMIT) {
                    if (isFloatValue(linear->potentiometer) && !linear->initialized) {
                        linear->sensorless = true;
                        linear->error = PotentiometerError;
                    }
                    else if (linear->max > 800 && linear->speed > 0.0f && potentData >= linear->max - 20) {
                        linear->atMax = true;
                        linear->timeWithoutChange = 0;
                    }
                    else if (linear->min < 200 && linear->speed < 0.0f && potentData <= linear->min + 20) {
                        linear->atMin = true;
                        linear->timeWithoutChange = 0;
                    }
                    else {
                        if (linear->error == None || linear->error == ActuatorsSyncError) {
                            if(linear->initialized && isFloatValue(potentData)){
                                linear->sensorless = true;
                                linear->error = PotentiometerError;
                                errorLogged = true;
                            }
                            else{
                                linear->error = ActuatorNotMovingError;
                                errorLogged = true;
                            }
                        }
                    }
                }
            }
        }
        else {
            linear->timeWithoutChange = 0;
            if (linear->error == ActuatorNotMovingError) linear->error = None;
            if (linear->atMax && linear->speed < 0.0f) linear->atMax = false;
            if (linear->atMin && linear->speed > 0.0f) linear->atMin = false;
        }

        linear->potentiometer = potentData;

        // Special case for bucket actuators
        if (linear->motorNumber == 16 || linear->motorNumber == 17) {
            if (potentData > 700) linear->atMax = true;
            else linear->atMax = false;
        }
        return errorLogged;
    }

    /*
    Extending:
    Upper += Time * upperSpeed;
    Est += Time * (lowerSpeed + (expMaxCurr - current) * (upperSpeed - lowerSpeed))
    Lower += Time * lowerSpeed;

    Retracting:
    Upper -= Time * lowerSpeed;
    Est -= Time * (lowerSpeed + (expMaxCurr - current) * (upperSpeed - lowerSpeed))
    Lower -= Time * upperSpeed;

    The upper and lower speeds are given by the datasheet.
    The estimate relies on using the current to estimate load and speed of the motor
    to get a better estimate of what the current position is. 
    These upper and lower estimates should bound the possible positions for the
    actuator based on the max and min speeds of the motor. These bounds will
    grow smaller when the actuator reaches end of travel in either direction.
    */
    inline void updateMotorPosition(int millis, LinearActuator *linear, bool isSystemRunning){
        if(isSystemRunning){
            linear->distance = linear->speed * linear->extensionSpeed * (millis / 1000.0) + linear->distance;
        }
        
        // Clamping logic
        if(linear->distance > linear->stroke){
            linear->distance = linear->stroke;
            linear->atMax = true;
        }
        else if(linear->distance < 0.0){
            linear->distance = 0.0;
            linear->atMin = true;
        }
        else{
            linear->atMin = false;
            linear->atMax = false;
        }
    }

    /** @brief Function that sets the speeds of the first pair of linear
     * actuators, then syncs the motors. 
     * 
     * The setSpeed function checks if the linear actuators are at the min
     * or max, then sets the speed to 0.0 if either are true.
     * @return void
     * */
    inline void setSpeedsPair(LinearActuator* a, LinearActuator* b, float currentSpeed, bool automationGo) {
        if (!automationGo) {
            a->speed = currentSpeed;
            b->speed = currentSpeed;
        } else {
            if (a->error != PotentiometerError && b->error != PotentiometerError) {
                a->speed = currentSpeed;
                b->speed = currentSpeed;
            }
        }

        if (a->error != PotentiometerError && b->error != PotentiometerError) {
            // Use the sync logic defined earlier
            sync(a, b, currentSpeed);
            setSpeedAtEnd(a, currentSpeed);
            setSpeedAtEnd(b, currentSpeed);
        }
    }

    /** @brief Function that sets the speeds of the first pair of linear
     * actuators, then syncs the motors.
     * 
     * The setSpeed function checks if the linear actuators are at the min
     * or max, then sets the speed to 0.0 if either are true. The values are
     * published if either is not zero or the currentSpeed is not zero.
     * @return void
     * */
    inline void setSpeedsDistancePair(LinearActuator* a, LinearActuator* b, float currentSpeed) {
        a->speed = currentSpeed;
        b->speed = currentSpeed;
        syncDistance(a, b, currentSpeed);
        setSpeedAtEnd(a, currentSpeed);
        setSpeedAtEnd(b, currentSpeed);
    }

    /** @brief Function that checks if the linear actuators are out of sync
     * then sets the error state to the correct one
     * 
     * The function checks if the difference between the potentiometers is 
     * greater than the thresh1 value, then checks if the error state is 
     * None. If the error is None, the error is set to ActuatorsSyncError, 
     * which indicates that the actuators are out of sync.
     * @return void
     * */
    inline bool setSyncErrors(LinearActuator* a, LinearActuator* b, float currentSpeed) {
        float diff = std::abs(a->potentiometer - b->potentiometer);
        float thresh = (950.0f / a->stroke) / 6.0f;

        if (diff > thresh) {
            // Check if floating (disconnected)
            if (isFloatValue(a->potentiometer) && !a->initialized) { 
                a->error = PotentiometerError; 
                a->sensorless = true; 
            }
            if (isFloatValue(b->potentiometer) && !b->initialized) { 
                b->error = PotentiometerError; 
                b->sensorless = true; 
            }
            
            if (a->error == None) a->error = ActuatorsSyncError;
            if (b->error == None) b->error = ActuatorsSyncError;
        }
        else {
            if (a->error == ActuatorsSyncError) a->error = None;
            if (b->error == ActuatorsSyncError) b->error = None;
        }

        // Capture speeds before sync
        float prevA = a->speed;
        float prevB = b->speed;

        if (a->error != PotentiometerError && b->error != PotentiometerError) {
            sync(a, b, currentSpeed);
        }

        // Return true if speeds changed
        return (a->speed != prevA || b->speed != prevB);
    }

    /** @brief Function to set potentiometer error.
     * 
     * This function is used to set the value of the error
     * of the linear object.  If the potentiometer is equal
     * to 1024, which is the value that occurs when the
     * potentiometer is disconnected from the Arduino. Refer
     * to the ErrorState state diagram for more information.
     * @param potentData - Int value of potentiometer
     * @param *linear - Pointer to linear object
     * @return void
     * */
    inline bool updateActuatorFromSensor(int position, float current, LinearActuator* linear, bool isSystemRunning) {
        linear->maxCurrent = current;
        
        if (linear->sensorless) return false;

        // 1024 check
        if (position > 1024) {
            linear->error = PotentiometerError;
            linear->sensorless = true;
            return true; // Error occurred
        }

        // Clear floating error
        if (position > POT_FLOAT_HIGH || position < POT_FLOAT_LOW) {
            if (linear->error == PotentiometerError) linear->error = None;
        }

        if (linear->error != PotentiometerError) {
            return processPotentiometer(position, linear, isSystemRunning);
        }
        return false;
    }
}

#endif