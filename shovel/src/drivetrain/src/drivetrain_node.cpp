#include <cmath>
#include <string>
#include <chrono>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/empty.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <messages/msg/key_state.hpp>

#include "messages/msg/linear_status.hpp"
#include "messages/msg/falcon_status.hpp"
#include "messages/msg/kraken_status.hpp"
#include "messages/msg/drivetrain_status.hpp"
#include "utils/utils.hpp"


/** @file
 * @brief Drivetrain node — manages drive motor speed routing, slip detection,
 *        wheel odometry computation, and cmd_vel conversion for Nav2 integration.
 *
 * This node supports a mixed drivetrain where some wheels may use Falcon 500
 * motors (Phoenix 5 API) and others may use Kraken x60 motors (Phoenix 6 API).
 * Each wheel's motor type is specified via parameters, and the node handles
 * the unit conversion differences internally.
 *
 * Subscribed topics:
 *   - drive_left_speed (Float32)   — autonomy left side command
 *   - drive_right_speed (Float32)  — autonomy right side command
 *   - user_left_speed (Float32)    — manual left side command
 *   - user_right_speed (Float32)   — manual right side command
 *   - cmd_vel (Twist)              — Nav2 velocity command
 *   - talon_10_info (FalconStatus or KrakenStatus) — right front motor
 *   - talon_11_info (FalconStatus or KrakenStatus) — left front motor
 *   - talon_12_info (FalconStatus or KrakenStatus) — right rear motor
 *   - talon_13_info (FalconStatus or KrakenStatus) — left rear motor
 *
 * Published topics:
 *   - falcon_10_speed (Float32)    — right front motor speed
 *   - falcon_11_speed (Float32)    — left front motor speed
 *   - falcon_12_speed (Float32)    — right rear motor speed
 *   - falcon_13_speed (Float32)    — left rear motor speed
 *   - falcon_10_user_speed (Float32) — right front user speed
 *   - falcon_11_user_speed (Float32) — left front user speed
 *   - falcon_12_user_speed (Float32) — right rear user speed
 *   - falcon_13_user_speed (Float32) — left rear user speed
 *   - drivetrain_status (DrivetrainStatus)
 *   - odom (Odometry)              — wheel odometry for Nav2
 *   - TF: odom -> base_link
 *
 * Parameters:
 *   - wheel_diameter (double, default 0.2)     — wheel diameter in meters
 *   - gear_reduction (double, default 100.0)   — motor to wheel gear ratio
 *   - track_width (double, default 0.6)        — distance between left/right wheel centers in meters
 *   - motor0_type (string, default "phoenix6") — right front motor type ("phoenix5" or "phoenix6")
 *   - motor1_type (string, default "phoenix6") — left front motor type
 *   - motor2_type (string, default "phoenix6") — right rear motor type
 *   - motor3_type (string, default "phoenix6") — left rear motor type
 *   - use_cmd_vel (bool, default false)        — enable cmd_vel subscriber for Nav2
 *   - max_linear_speed (double, default 0.5)   — max linear speed in m/s for cmd_vel scaling
 *   - print_data (bool, default false)         — enable verbose logging
 */


// ============================================================================
//  Motor type abstraction
// ============================================================================

enum class MotorType {
    PHOENIX_5,  // Falcon 500: 2048 units/rev, velocity in units/100ms
    PHOENIX_6   // Kraken x60: velocity in rotations/sec, position in rotations
};

static constexpr double PHOENIX5_UNITS_PER_REV = 2048.0;
static constexpr double PHOENIX5_VEL_TO_RPS = 10.0 / PHOENIX5_UNITS_PER_REV;  // units/100ms -> rev/sec

struct WheelState {
    MotorType motorType = MotorType::PHOENIX_6;

    // Raw values from the motor controller
    double rawVelocity = 0.0;
    double rawPosition = 0.0;

    // Converted values in ground units
    double groundSpeed = 0.0;     // m/s
    double groundPosition = 0.0;  // meters of travel

    // Additional telemetry
    double rpm = 0.0;
    bool updated = false;

    void updateFromFalcon(double sensorVelocity, double sensorPosition,
                          double gearReduction, double wheelCircum) {
        rawVelocity = sensorVelocity;
        rawPosition = sensorPosition;

        // Phoenix 5: velocity is in raw units per 100ms
        // Convert to motor RPS, then to wheel RPS via gear reduction
        double motorRPS = sensorVelocity * PHOENIX5_VEL_TO_RPS;
        double wheelRPS = motorRPS / gearReduction;
        rpm = wheelRPS * 60.0;
        groundSpeed = wheelRPS * wheelCircum;
        groundPosition = (sensorPosition / PHOENIX5_UNITS_PER_REV / gearReduction) * wheelCircum;
        updated = true;
    }

    void updateFromKraken(double velocity, double position,
                          double gearReduction, double wheelCircum) {
        rawVelocity = velocity;
        rawPosition = position;

        // Phoenix 6: velocity is in rotations/sec, position in rotations
        double wheelRPS = velocity / gearReduction;
        rpm = wheelRPS * 60.0;
        groundSpeed = wheelRPS * wheelCircum;
        groundPosition = (position / gearReduction) * wheelCircum;
        updated = true;
    }
};


// ============================================================================
//  Globals
// ============================================================================

rclcpp::Node::SharedPtr nodeHandle;

// Motor speed publishers (autonomy)
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> falcon10Publisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> falcon11Publisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> falcon12Publisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> falcon13Publisher;

// Motor speed publishers (user/manual)
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> falcon10UserPublisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> falcon11UserPublisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> falcon12UserPublisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> falcon13UserPublisher;

// Status publishers
std::shared_ptr<rclcpp::Publisher<messages::msg::DrivetrainStatus>> drivetrainStatusPublisher;
std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odomPublisher;

// TF broadcaster
std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;

// Wheel states: 0=right front (10), 1=left front (11), 2=right rear (12), 3=left rear (13)
WheelState wheels[4];

// Drivetrain parameters
double wheelDiameter = 0.2;
double wheelCircum = 0.0;
double wheelRadius = 0.0;
double gearReduction = 100.0;
double trackWidth = 0.6;
bool printData = false;
bool useCmdVel = false;
bool useSim = false;
double maxLinearSpeed = 0.5;

// Odometry state
double odomX = 0.0;
double odomY = 0.0;
double odomTheta = 0.0;
double prevLeftPos = 0.0;
double prevRightPos = 0.0;
bool odomInitialized = false;
bool publishOdom = true;

// Slip detection
const double SLIP_THRESHOLD = 0.3;
const double SLIP_CLAMP_FACTOR = 0.8;
float lastLeftSpeed = 0.0;
float lastRightSpeed = 0.0;

// Simulated encoder values
double simLeftPos = 0.0;
double simRightPos = 0.0;
rclcpp::Time lastUpdateTime;


/** @brief Joint state callback — reads wheel positions/velocities from Gazebo's
 *        ros2_control joint_state_broadcaster.
 *
 * In simulation, the physics engine is the ground truth. Gazebo reports joint
 * positions in radians and velocities in rad/s. We convert to linear meters
 * and m/s by multiplying by the wheel radius.
 *
 * Note on BR_Wheel_Joint: The URDF defines this joint with axis (0, -1, 0)
 * and a pi rotation on the joint origin. Gazebo may report the velocity with
 * the opposite sign for this wheel. We negate it to keep the convention that
 * positive velocity = forward motion.
 */
void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    for (size_t i = 0; i < msg->name.size(); i++) {
        // Gazebo joint positions are in radians, velocities in rad/s.
        // Multiply by wheel radius to get linear distance/speed at the ground.
        double pos_meters = msg->position[i] * wheelRadius;
        double vel_meters = msg->velocity[i] * wheelRadius;

        if (msg->name[i] == "FR_Wheel_Joint") {
            wheels[0].groundPosition = pos_meters;
            wheels[0].groundSpeed = vel_meters;
            wheels[0].updated = true;
        } else if (msg->name[i] == "FL_Wheel_Joint") {
            wheels[1].groundPosition = pos_meters;
            wheels[1].groundSpeed = vel_meters;
            wheels[1].updated = true;
        } else if (msg->name[i] == "BR_Wheel_Joint") {
            // Negate: URDF axis is (0, -1, 0) with pi rotation
            wheels[2].groundPosition = -pos_meters;
            wheels[2].groundSpeed = -vel_meters;
            wheels[2].updated = true;
        } else if (msg->name[i] == "BL_Wheel_Joint") {
            wheels[3].groundPosition = pos_meters;
            wheels[3].groundSpeed = vel_meters;
            wheels[3].updated = true;
        }
    }
}


void driveLeftSpeedCallback(const std_msgs::msg::Float32::SharedPtr speed) {
    if (printData)
        RCLCPP_INFO(nodeHandle->get_logger(), "driveLeftSpeed: %f", speed->data);

    lastLeftSpeed = speed->data;
    std_msgs::msg::Float32 outSpeed;
    outSpeed.data = speed->data;
    falcon11Publisher->publish(outSpeed);
    falcon13Publisher->publish(outSpeed);
}


void driveRightSpeedCallback(const std_msgs::msg::Float32::SharedPtr speed) {
    if (printData)
        RCLCPP_INFO(nodeHandle->get_logger(), "driveRightSpeed: %f", speed->data);

    lastRightSpeed = speed->data;
    std_msgs::msg::Float32 outSpeed;
    outSpeed.data = speed->data;
    falcon10Publisher->publish(outSpeed);
    falcon12Publisher->publish(outSpeed);
}


void userLeftSpeedCallback(const std_msgs::msg::Float32::SharedPtr speed) {
    if (printData)
        RCLCPP_INFO(nodeHandle->get_logger(), "userLeftSpeed: %f", speed->data);

    lastLeftSpeed = speed->data;
    std_msgs::msg::Float32 outSpeed;
    outSpeed.data = speed->data;
    falcon11UserPublisher->publish(outSpeed);
    falcon13UserPublisher->publish(outSpeed);
}


void userRightSpeedCallback(const std_msgs::msg::Float32::SharedPtr speed) {
    if (printData)
        RCLCPP_INFO(nodeHandle->get_logger(), "userRightSpeed: %f", speed->data);

    lastRightSpeed = speed->data;
    std_msgs::msg::Float32 outSpeed;
    outSpeed.data = speed->data;
    falcon10UserPublisher->publish(outSpeed);
    falcon12UserPublisher->publish(outSpeed);
}


/** @brief cmd_vel callback — converts Nav2 Twist commands to differential drive speeds.
 *
 * Nav2 publishes geometry_msgs/Twist with linear.x (m/s) and angular.z (rad/s).
 * We convert to left/right duty cycle percentages using the differential drive
 * kinematics and publish to the individual motor speed topics.
 *
 * The conversion from m/s to duty cycle percentage is linear, scaled by
 * maxLinearSpeed. This is a simplification — for precise velocity control
 * you'd want closed-loop velocity mode on the motor controllers.
 */
void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr twist) {
    if (!useCmdVel) return;

    double linearX = twist->linear.x;   // m/s
    double angularZ = twist->angular.z;  // rad/s

    // Differential drive inverse kinematics
    // v_left  = linear - (angular * trackWidth / 2)
    // v_right = linear + (angular * trackWidth / 2)
    double leftVel  = linearX - (angularZ * trackWidth / 2.0);
    double rightVel = linearX + (angularZ * trackWidth / 2.0);

    // Convert m/s to duty cycle percentage [-1.0, 1.0]
    // Clamp to prevent exceeding motor limits
    double leftPercent  = std::clamp(leftVel / maxLinearSpeed, -1.0, 1.0);
    double rightPercent = std::clamp(rightVel / maxLinearSpeed, -1.0, 1.0);

    if (printData) {
        RCLCPP_INFO(nodeHandle->get_logger(),
                     "cmd_vel: lin=%.3f ang=%.3f -> L=%.3f R=%.3f",
                     linearX, angularZ, leftPercent, rightPercent);
    }

    lastLeftSpeed = leftPercent;
    lastRightSpeed = rightPercent;

    std_msgs::msg::Float32 leftMsg, rightMsg;
    leftMsg.data = leftPercent;
    rightMsg.data = rightPercent;

    falcon11Publisher->publish(leftMsg);
    falcon13Publisher->publish(leftMsg);
    falcon10Publisher->publish(rightMsg);
    falcon12Publisher->publish(rightMsg);
}


void falcon0Callback(const messages::msg::FalconStatus::SharedPtr status) {
    wheels[0].updateFromFalcon(status->sensor_velocity, status->sensor_position,
                               gearReduction, wheelCircum);
}

void falcon1Callback(const messages::msg::FalconStatus::SharedPtr status) {
    wheels[1].updateFromFalcon(status->sensor_velocity, status->sensor_position,
                               gearReduction, wheelCircum);
}

void falcon2Callback(const messages::msg::FalconStatus::SharedPtr status) {
    wheels[2].updateFromFalcon(status->sensor_velocity, status->sensor_position,
                               gearReduction, wheelCircum);
}

void falcon3Callback(const messages::msg::FalconStatus::SharedPtr status) {
    wheels[3].updateFromFalcon(status->sensor_velocity, status->sensor_position,
                               gearReduction, wheelCircum);
}


void kraken0Callback(const messages::msg::KrakenStatus::SharedPtr status) {
    wheels[0].updateFromKraken(status->sensor_velocity, status->sensor_position,
                               gearReduction, wheelCircum);
}

void kraken1Callback(const messages::msg::KrakenStatus::SharedPtr status) {
    wheels[1].updateFromKraken(status->sensor_velocity, status->sensor_position,
                               gearReduction, wheelCircum);
}

void kraken2Callback(const messages::msg::KrakenStatus::SharedPtr status) {
    wheels[2].updateFromKraken(status->sensor_velocity, status->sensor_position,
                               gearReduction, wheelCircum);
}

void kraken3Callback(const messages::msg::KrakenStatus::SharedPtr status) {
    wheels[3].updateFromKraken(status->sensor_velocity, status->sensor_position,
                               gearReduction, wheelCircum);
}


void checkAndLimitSlip() {
    if (std::abs(lastLeftSpeed) < 0.01 && std::abs(lastRightSpeed) < 0.01)
        return;

    // Right side: wheels[0] (front) and wheels[2] (rear)
    // Left side:  wheels[1] (front) and wheels[3] (rear)
    double rightFront = std::abs(wheels[0].groundSpeed);
    double rightRear  = std::abs(wheels[2].groundSpeed);
    double leftFront  = std::abs(wheels[1].groundSpeed);
    double leftRear   = std::abs(wheels[3].groundSpeed);

    // Check right side
    if (std::abs(lastRightSpeed) > 0.01) {
        double rightRef = std::min(rightFront, rightRear);
        if (rightRef > 0.01) {
            if ((rightFront - rightRef) / rightRef > SLIP_THRESHOLD) {
                RCLCPP_WARN(nodeHandle->get_logger(),
                            "Right front slipping! Speed: %.3f, Ref: %.3f",
                            rightFront, rightRef);
                // TODO: Uncomment when slip correction is validated
                // std_msgs::msg::Float32 reduced;
                // reduced.data = lastRightSpeed * SLIP_CLAMP_FACTOR;
                // falcon10Publisher->publish(reduced);
            }
            if ((rightRear - rightRef) / rightRef > SLIP_THRESHOLD) {
                RCLCPP_WARN(nodeHandle->get_logger(),
                            "Right rear slipping! Speed: %.3f, Ref: %.3f",
                            rightRear, rightRef);
            }
        }
    }

    // Check left side
    if (std::abs(lastLeftSpeed) > 0.01) {
        double leftRef = std::min(leftFront, leftRear);
        if (leftRef > 0.01) {
            if ((leftFront - leftRef) / leftRef > SLIP_THRESHOLD) {
                RCLCPP_WARN(nodeHandle->get_logger(),
                            "Left front slipping! Speed: %.3f, Ref: %.3f",
                            leftFront, leftRef);
            }
            if ((leftRear - leftRef) / leftRef > SLIP_THRESHOLD) {
                RCLCPP_WARN(nodeHandle->get_logger(),
                            "Left rear slipping! Speed: %.3f, Ref: %.3f",
                            leftRear, leftRef);
            }
        }
    }
}


/** @brief Compute and publish wheel odometry + TF.
 *
 * Uses differential drive forward kinematics:
 *   dCenter = (dLeft + dRight) / 2
 *   dTheta  = (dRight - dLeft) / trackWidth
 *   x += dCenter * cos(theta + dTheta/2)
 *   y += dCenter * sin(theta + dTheta/2)
 *
 * The midpoint angle (theta + dTheta/2) provides a better approximation
 * for arc-based motion than using the previous theta alone.
 */
void updateOdometry() {
    // Average left side (wheels 1, 3) and right side (wheels 0, 2)
    double leftPos  = (wheels[0].groundPosition + wheels[2].groundPosition) / 2.0;
    double rightPos = (wheels[1].groundPosition + wheels[3].groundPosition) / 2.0;

    if (!odomInitialized) {
        prevLeftPos = leftPos;
        prevRightPos = rightPos;
        odomInitialized = true;
        return;
    }

    double dLeft  = leftPos - prevLeftPos;
    double dRight = rightPos - prevRightPos;
    prevLeftPos  = leftPos;
    prevRightPos = rightPos;

    // Skip if no movement (avoids publishing stale transforms)
    if (std::abs(dLeft) < 1e-8 && std::abs(dRight) < 1e-8) {
        // Still publish odom at current position for Nav2 freshness
    }

    double dCenter = (dLeft + dRight) / 2.0;
    double dTheta  = (dRight - dLeft) / trackWidth;

    // Update pose using midpoint angle for better arc approximation
    odomX     += dCenter * cos(odomTheta + dTheta / 2.0);
    odomY     += dCenter * sin(odomTheta + dTheta / 2.0);
    odomTheta += dTheta;

    // Normalize theta to [-pi, pi]
    while (odomTheta > M_PI)  odomTheta -= 2.0 * M_PI;
    while (odomTheta < -M_PI) odomTheta += 2.0 * M_PI;

    // Velocity from wheel speeds
    double leftVel  = (wheels[1].groundSpeed + wheels[3].groundSpeed) / 2.0;
    double rightVel = (wheels[0].groundSpeed + wheels[2].groundSpeed) / 2.0;
    double linearVel  = (leftVel + rightVel) / 2.0;
    double angularVel = (rightVel - leftVel) / trackWidth;

    auto now = nodeHandle->get_clock()->now();

    // --- Publish nav_msgs/Odometry ---
    nav_msgs::msg::Odometry odomMsg;
    odomMsg.header.stamp = now;
    odomMsg.header.frame_id = "odom";
    odomMsg.child_frame_id = "base_link";

    odomMsg.pose.pose.position.x = odomX;
    odomMsg.pose.pose.position.y = odomY;
    odomMsg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, odomTheta);
    odomMsg.pose.pose.orientation.x = q.x();
    odomMsg.pose.pose.orientation.y = q.y();
    odomMsg.pose.pose.orientation.z = q.z();
    odomMsg.pose.pose.orientation.w = q.w();

    // Pose covariance — diagonal values only for now.
    // These should be tuned based on testing. Higher values = less trust.
    // Order: x, y, z, roll, pitch, yaw
    odomMsg.pose.covariance[0]  = 0.01;   // x
    odomMsg.pose.covariance[7]  = 0.01;   // y
    odomMsg.pose.covariance[14] = 1e6;    // z (we don't measure this)
    odomMsg.pose.covariance[21] = 1e6;    // roll
    odomMsg.pose.covariance[28] = 1e6;    // pitch
    odomMsg.pose.covariance[35] = 0.03;   // yaw

    odomMsg.twist.twist.linear.x = linearVel;
    odomMsg.twist.twist.linear.y = 0.0;
    odomMsg.twist.twist.angular.z = angularVel;

    // Twist covariance
    odomMsg.twist.covariance[0]  = 0.01;   // linear x
    odomMsg.twist.covariance[7]  = 1e6;    // linear y
    odomMsg.twist.covariance[14] = 1e6;    // linear z
    odomMsg.twist.covariance[21] = 1e6;    // angular x
    odomMsg.twist.covariance[28] = 1e6;    // angular y
    odomMsg.twist.covariance[35] = 0.03;   // angular z

    if (publishOdom) {
        odomPublisher->publish(odomMsg);

        // --- Broadcast TF: odom -> base_link ---
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = now;
        tf.header.frame_id = "odom";
        tf.child_frame_id = "base_link";
        tf.transform.translation.x = odomX;
        tf.transform.translation.y = odomY;
        tf.transform.translation.z = 0.0;
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf.transform.rotation.w = q.w();

        tfBroadcaster->sendTransform(tf);
    }

    if (printData) {
        RCLCPP_INFO(nodeHandle->get_logger(),
                     "Odom: x=%.3f y=%.3f theta=%.2f deg | vel: lin=%.3f ang=%.3f",
                     odomX, odomY, odomTheta * 180.0 / M_PI,
                     linearVel, angularVel);
    }
}


void publishStatus() {
    messages::msg::DrivetrainStatus status;

    status.falcon1_velocity     = wheels[0].rawVelocity;
    status.falcon1_rpm          = wheels[0].rpm;
    status.falcon1_ground_speed = wheels[0].groundSpeed;

    status.falcon2_velocity     = wheels[1].rawVelocity;
    status.falcon2_rpm          = wheels[1].rpm;
    status.falcon2_ground_speed = wheels[1].groundSpeed;

    status.falcon3_velocity     = wheels[2].rawVelocity;
    status.falcon3_rpm          = wheels[2].rpm;
    status.falcon3_ground_speed = wheels[2].groundSpeed;

    status.falcon4_velocity     = wheels[3].rawVelocity;
    status.falcon4_rpm          = wheels[3].rpm;
    status.falcon4_ground_speed = wheels[3].groundSpeed;

    drivetrainStatusPublisher->publish(status);
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    nodeHandle = rclcpp::Node::make_shared("drivetrain");

    // --- Parameters ---
    wheelDiameter  = utils::getParameter<double>(nodeHandle, "wheel_diameter", 0.2);
    gearReduction  = utils::getParameter<double>(nodeHandle, "gear_reduction", 100.0);
    trackWidth     = utils::getParameter<double>(nodeHandle, "track_width", 0.6);
    useCmdVel      = utils::getParameter<bool>(nodeHandle, "use_cmd_vel", false);
    publishOdom    = utils::getParameter<bool>(nodeHandle, "publish_odom", true);
    maxLinearSpeed = utils::getParameter<double>(nodeHandle, "max_linear_speed", 0.5);
    printData      = utils::getParameter<bool>(nodeHandle, "print_data", false);
    useSim         = utils::getParameter<bool>(nodeHandle, "use_sim", false);
    wheelCircum    = wheelDiameter * M_PI;
    wheelRadius    = wheelDiameter / 2.0;

    // Motor types per wheel: "phoenix5" for Falcon 500, "phoenix6" for Kraken x60
    std::string motor0Type = utils::getParameter<std::string>(nodeHandle, "motor0_type", "phoenix6");
    std::string motor1Type = utils::getParameter<std::string>(nodeHandle, "motor1_type", "phoenix6");
    std::string motor2Type = utils::getParameter<std::string>(nodeHandle, "motor2_type", "phoenix6");
    std::string motor3Type = utils::getParameter<std::string>(nodeHandle, "motor3_type", "phoenix6");

    wheels[0].motorType = (motor0Type == "phoenix5") ? MotorType::PHOENIX_5 : MotorType::PHOENIX_6;
    wheels[1].motorType = (motor1Type == "phoenix5") ? MotorType::PHOENIX_5 : MotorType::PHOENIX_6;
    wheels[2].motorType = (motor2Type == "phoenix5") ? MotorType::PHOENIX_5 : MotorType::PHOENIX_6;
    wheels[3].motorType = (motor3Type == "phoenix5") ? MotorType::PHOENIX_5 : MotorType::PHOENIX_6;

    RCLCPP_INFO(nodeHandle->get_logger(), "Drivetrain config:");
    RCLCPP_INFO(nodeHandle->get_logger(), "  wheel_diameter: %.3f m", wheelDiameter);
    RCLCPP_INFO(nodeHandle->get_logger(), "  gear_reduction: %.1f:1", gearReduction);
    RCLCPP_INFO(nodeHandle->get_logger(), "  track_width:    %.3f m", trackWidth);
    RCLCPP_INFO(nodeHandle->get_logger(), "  motor types: [%s, %s, %s, %s]",
                motor0Type.c_str(), motor1Type.c_str(),
                motor2Type.c_str(), motor3Type.c_str());
    RCLCPP_INFO(nodeHandle->get_logger(), "  use_cmd_vel: %s", useCmdVel ? "true" : "false");
    RCLCPP_INFO(nodeHandle->get_logger(), "  use_sim:     %s", useSim ? "true" : "false");

    // --- Speed command subscribers ---
    auto driveLeftSub  = nodeHandle->create_subscription<std_msgs::msg::Float32>(
        "drive_left_speed", 1, driveLeftSpeedCallback);
    auto driveRightSub = nodeHandle->create_subscription<std_msgs::msg::Float32>(
        "drive_right_speed", 1, driveRightSpeedCallback);
    auto userLeftSub   = nodeHandle->create_subscription<std_msgs::msg::Float32>(
        "user_left_speed", 1, userLeftSpeedCallback);
    auto userRightSub  = nodeHandle->create_subscription<std_msgs::msg::Float32>(
        "user_right_speed", 1, userRightSpeedCallback);

    // Nav2 cmd_vel subscriber
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub;
    if (useCmdVel) {
        cmdVelSub = nodeHandle->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 1, cmdVelCallback);
        RCLCPP_INFO(nodeHandle->get_logger(), "cmd_vel subscriber active");
    }

    // Sim mode: read wheel states from Gazebo's joint_state_broadcaster
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStateSub;
    if (useSim) {
        jointStateSub = nodeHandle->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, jointStateCallback);
        RCLCPP_INFO(nodeHandle->get_logger(), "Sim mode: reading odometry from joint_states");
    }

    // --- Motor status subscribers ---
    // Create the appropriate subscriber type per wheel based on motor type.
    // We hold them in SharedPtrs to keep them alive.
    // Using rclcpp::SubscriptionBase::SharedPtr to hold either type.
    std::vector<rclcpp::SubscriptionBase::SharedPtr> motorSubs;

    // Wheel 0 — right front (talon_10_info)
    if (wheels[0].motorType == MotorType::PHOENIX_5) {
        motorSubs.push_back(nodeHandle->create_subscription<messages::msg::FalconStatus>(
            "talon_10_info", 1, falcon0Callback));
    } else {
        motorSubs.push_back(nodeHandle->create_subscription<messages::msg::KrakenStatus>(
            "talon_10_info", 1, kraken0Callback));
    }

    // Wheel 1 — left front (talon_11_info)
    if (wheels[1].motorType == MotorType::PHOENIX_5) {
        motorSubs.push_back(nodeHandle->create_subscription<messages::msg::FalconStatus>(
            "talon_11_info", 1, falcon1Callback));
    } else {
        motorSubs.push_back(nodeHandle->create_subscription<messages::msg::KrakenStatus>(
            "talon_11_info", 1, kraken1Callback));
    }

    // Wheel 2 — right rear (talon_12_info)
    if (wheels[2].motorType == MotorType::PHOENIX_5) {
        motorSubs.push_back(nodeHandle->create_subscription<messages::msg::FalconStatus>(
            "talon_12_info", 1, falcon2Callback));
    } else {
        motorSubs.push_back(nodeHandle->create_subscription<messages::msg::KrakenStatus>(
            "talon_12_info", 1, kraken2Callback));
    }

    // Wheel 3 — left rear (talon_13_info)
    if (wheels[3].motorType == MotorType::PHOENIX_5) {
        motorSubs.push_back(nodeHandle->create_subscription<messages::msg::FalconStatus>(
            "talon_13_info", 1, falcon3Callback));
    } else {
        motorSubs.push_back(nodeHandle->create_subscription<messages::msg::KrakenStatus>(
            "talon_13_info", 1, kraken3Callback));
    }

    // --- Publishers ---
    falcon10Publisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("falcon_10_speed", 1);
    falcon11Publisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("falcon_11_speed", 1);
    falcon12Publisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("falcon_12_speed", 1);
    falcon13Publisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("falcon_13_speed", 1);

    falcon10UserPublisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("falcon_10_user_speed", 1);
    falcon11UserPublisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("falcon_11_user_speed", 1);
    falcon12UserPublisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("falcon_12_user_speed", 1);
    falcon13UserPublisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("falcon_13_user_speed", 1);

    drivetrainStatusPublisher = nodeHandle->create_publisher<messages::msg::DrivetrainStatus>("drivetrain_status", 1);
    odomPublisher = nodeHandle->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    // TF broadcaster
    tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(nodeHandle);

    RCLCPP_INFO(nodeHandle->get_logger(), "Drivetrain node initialized");
    lastUpdateTime = nodeHandle->get_clock()->now();

    // --- Main loop ---
    rclcpp::Rate rate(60);
    while (rclcpp::ok()) {
        auto currentTime = nodeHandle->get_clock()->now();
        double dt = (currentTime - lastUpdateTime).seconds();
        lastUpdateTime = currentTime;

        updateOdometry();
        publishStatus();
        checkAndLimitSlip();
        rate.sleep();
        rclcpp::spin_some(nodeHandle);
    }

    rclcpp::shutdown();
    return 0;
}