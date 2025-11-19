#include <gtest/gtest.h>
#include "excavation_core.hpp"

/*
Run the following commands:
colcon build --packages-select excavation
colcon test --packages-select excavation
colcon test-result --verbose
*/

class ExcavationFuncTest : public ::testing::Test {
protected:
    core::LinearActuator createDefault(int id) {
        return core::LinearActuator(id, 10.0f, 1.0f, 10.0f);
    }
};

// 1. Test setSpeedAtEnd
TEST_F(ExcavationFuncTest, SpeedStopsAtMax) {
    core::LinearActuator a = createDefault(14);
    a.atMax = true;
    a.speed = 1.0f; // Trying to extend
    
    // CORRECTED: core::setSpeedAtEnd
    core::setSpeedAtEnd(&a, 1.0f);
    
    EXPECT_FLOAT_EQ(a.speed, 0.0f); // Should stop
}

TEST_F(ExcavationFuncTest, SpeedAllowedRetractAtMax) {
    core::LinearActuator a = createDefault(14);
    a.atMax = true;
    a.speed = -1.0f; // Trying to retract
    
    core::setSpeedAtEnd(&a, -1.0f);
    
    EXPECT_FLOAT_EQ(a.speed, -1.0f); // Should NOT stop
}

// 2. Test updateMotorPosition
TEST_F(ExcavationFuncTest, PositionUpdatesBasedOnTime) {
    core::LinearActuator a = createDefault(14);
    a.distance = 5.0f;
    a.speed = 1.0f;
    a.extensionSpeed = 1.0f; // 1 inch per second
    bool run = true;
    
    // CORRECTED: core::updateMotorPosition
    core::updateMotorPosition(500, &a, run);
    
    // 5.0 + (1.0 * 1.0 * 0.5) = 5.5
    EXPECT_FLOAT_EQ(a.distance, 5.5f);
}

TEST_F(ExcavationFuncTest, PositionDoesNotUpdateIfPaused) {
    core::LinearActuator a = createDefault(14);
    a.distance = 5.0f;
    a.speed = 1.0f;
    bool run = false; // System paused
    
    core::updateMotorPosition(1000, &a, run);
    
    EXPECT_FLOAT_EQ(a.distance, 5.0f);
}

// 3. Test setSpeedsPair
TEST_F(ExcavationFuncTest, SpeedsPairIgnoresErrorsIfNoAutomation) {
    core::LinearActuator a = createDefault(14);
    core::LinearActuator b = createDefault(15);
    a.error = PotentiometerError; // Even with error
    
    bool automationGo = false;
    // CORRECTED: core::setSpeedsPair
    core::setSpeedsPair(&a, &b, 1.0f, automationGo);
    
    EXPECT_FLOAT_EQ(a.speed, 1.0f); // Should still set speed
}

TEST_F(ExcavationFuncTest, SpeedsPairRespectsAutomationSafety) {
    core::LinearActuator a = createDefault(14);
    core::LinearActuator b = createDefault(15);
    a.error = PotentiometerError;
    
    bool automationGo = true;
    // Initialize speeds to 0
    a.speed = 0.0f;
    
    core::setSpeedsPair(&a, &b, 1.0f, automationGo);
    
    // Should NOT set speed because of error + automation enabled
    EXPECT_FLOAT_EQ(a.speed, 0.0f); 
}

// 4. Test setSyncErrors
TEST_F(ExcavationFuncTest, SyncErrorsTriggersSpeedChange) {
    core::LinearActuator a = createDefault(14);
    core::LinearActuator b = createDefault(15);
    
    // Setup state where they are out of sync
    a.potentiometer = 600;
    b.potentiometer = 500; // Big difference
    a.speed = 1.0f;
    b.speed = 1.0f;
    
    // CORRECTED: core::setSyncErrors
    bool changed = core::setSyncErrors(&a, &b, 1.0f);
    
    EXPECT_TRUE(changed);
    EXPECT_NE(a.speed, 1.0f); // A should have slowed down
    EXPECT_EQ(a.error, ActuatorsSyncError); // Error should be set
}

// CORRECTED: Changed ExcavationLogicTest -> ExcavationFuncTest
TEST_F(ExcavationFuncTest, DetectsFloatingValues) {
    // CORRECTED: core::isFloatValue
    EXPECT_TRUE(core::isFloatValue(100));
    EXPECT_FALSE(core::isFloatValue(99));
}

TEST_F(ExcavationFuncTest, SyncSlowsFastMotorSlightly) {
    core::LinearActuator a = createDefault(14);
    core::LinearActuator b = createDefault(15);
    
    // Setup difference of 10 (approx 10% of stroke)
    a.potentiometer = 510;
    b.potentiometer = 500;
    a.speed = 1.0f;
    b.speed = 1.0f;

    // CORRECTED: core::sync
    core::sync(&a, &b, 1.0f);
    
    EXPECT_NEAR(a.speed, 0.9f, 0.001f);
    EXPECT_NEAR(b.speed, 1.0f, 0.001f);
}

TEST_F(ExcavationFuncTest, DetectsStall) {
    core::LinearActuator a = createDefault(14);
    a.initialized = true;
    a.potentiometer = 500;
    a.speed = 1.0f; // Should be moving
    bool run = true;

    // Loop 15 times with no change
    for(int i=0; i < 15; i++) {
        // CORRECTED: core::processPotentiometer
        core::processPotentiometer(500, &a, run);
    }

    EXPECT_EQ(a.error, ActuatorNotMovingError);
}