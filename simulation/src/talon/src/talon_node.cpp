#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <messages/msg/talon_status.hpp>
#include <map>
#include <string>
#include <cmath>
#include <algorithm>

static double clampd(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}

static constexpr int ARM_ID    = 14;  // "Talon 14" -> Arm actuator
static constexpr int BUCKET_ID = 15;  // "Talon 15" -> Bucket actuator

// Publish to Gazebo ros2_control position controllers
const std::map<int, std::string> ID_TO_GAZEBO_TOPIC = {
    {ARM_ID,    "/arm_position_controller/commands"},
    {BUCKET_ID, "/bucket_position_controller/commands"},
};

// Joint names in /joint_states to associate feedback with IDs
const std::map<std::string, int> JOINT_NAME_TO_ID = {
    {"Arm_Joint", ARM_ID},
    {"Bucket_Joint", BUCKET_ID},
};

// Input topics (from drivetrain/teleop layer)
// Keep your existing conventions: talon_14_speed etc.
// Interpret as "percent output" in [-1, 1]
static const std::string ARM_INPUT_TOPIC    = "talon_14_speed";
static const std::string BUCKET_INPUT_TOPIC = "talon_15_speed";

// --------------------------
// Actuator + joint constraints
// --------------------------
struct JointConfig {
    double lower_rad;
    double upper_rad;

    double stroke_in;              // inches
    double speed_in_per_s;         // inches/sec (max at |percent|=1)
};

static const JointConfig ARM_CFG{
    .lower_rad = -0.6,
    .upper_rad =  0.3,
    .stroke_in = 10.0,
    .speed_in_per_s = 0.5,
};

static const JointConfig BUCKET_CFG{
    .lower_rad = -1.25,
    .upper_rad =  0.45,
    .stroke_in = 4.0,
    .speed_in_per_s = 0.5,
};

class TalonSimNode : public rclcpp::Node {
public:
    TalonSimNode() : Node("talon_sim_node") {
        // 1) Publishers (Gazebo + Status)
        for (auto const& [id, topic] : ID_TO_GAZEBO_TOPIC) {
            gazebo_publishers_[id] = this->create_publisher<std_msgs::msg::Float64MultiArray>(topic, 10);

            std::string status_topic = "talon_" + std::to_string(id) + "_info";
            status_publishers_[id] = this->create_publisher<messages::msg::TalonStatus>(status_topic, 10);
        }

        // Initialize commanded positions at the LOWER limit (retracted)
        cmd_pos_rad_[ARM_ID]    = ARM_CFG.lower_rad;
        cmd_pos_rad_[BUCKET_ID] = BUCKET_CFG.lower_rad;

        // 2) Subscribe to user/teleop outputs (percent -1..1)
        sub_arm_ = this->create_subscription<std_msgs::msg::Float32>(
            ARM_INPUT_TOPIC, 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg) {
                arm_percent_ = clampd(msg->data, -1.0, 1.0);
            });

        sub_bucket_ = this->create_subscription<std_msgs::msg::Float32>(
            BUCKET_INPUT_TOPIC, 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg) {
                bucket_percent_ = clampd(msg->data, -1.0, 1.0);
            });

        // 3) Subscribe to joint feedback (optional but useful)
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&TalonSimNode::joint_state_callback, this, std::placeholders::_1));

        // 4) Update loop (50 Hz feels good for actuators)
        last_update_time_ = now();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&TalonSimNode::update_and_publish, this));

        RCLCPP_INFO(this->get_logger(), "Talon Arm/Bucket Simulation Bridge Started");
    }

private:
    struct SimMotorState {
        double position = 0.0;
        double velocity = 0.0;
        double effort   = 0.0;
    };

    // Feedback state from /joint_states
    std::map<int, SimMotorState> motor_states_;

    // Commanded position state (what we publish)
    std::map<int, double> cmd_pos_rad_;

    // Current “percent output” commands
    double arm_percent_ = 0.0;
    double bucket_percent_ = 0.0;

    rclcpp::Time last_update_time_;

    std::map<int, rclcpp::Publisher<messages::msg::TalonStatus>::SharedPtr> status_publishers_;
    std::map<int, rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr> gazebo_publishers_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_arm_, sub_bucket_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Convert actuator percent to delta angle using linear mapping:
    // inches/sec -> (rad/in) -> rad/sec, then integrate.
    static double rad_per_in(const JointConfig& cfg) {
        const double range = (cfg.upper_rad - cfg.lower_rad);
        return (cfg.stroke_in > 1e-9) ? (range / cfg.stroke_in) : 0.0;
    }

    static int pot_from_angle(double angle_rad, const JointConfig& cfg,
                          int pot_min = 20, int pot_max = 950) {
        const double range = (cfg.upper_rad - cfg.lower_rad);
        double u = 0.0;
        if (std::abs(range) > 1e-9) {
            u = (angle_rad - cfg.lower_rad) / range;
        }
        u = clampd(u, 0.0, 1.0);

        const double pot_f = pot_min + u * (pot_max - pot_min);
        int pot = static_cast<int>(std::lround(pot_f));
        pot = std::max(0, std::min(1024, pot));
        return pot;
    }


    void publish_joint_position_cmd(int id, double pos_rad) {
        std_msgs::msg::Float64MultiArray cmd;
        cmd.data = {pos_rad};
        auto it = gazebo_publishers_.find(id);
        if (it != gazebo_publishers_.end()) {
            it->second->publish(cmd);
        }
    }

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            auto it = JOINT_NAME_TO_ID.find(msg->name[i]);
            if (it == JOINT_NAME_TO_ID.end()) continue;

            int id = it->second;
            motor_states_[id].position = msg->position[i];

            if (i < msg->velocity.size()) motor_states_[id].velocity = msg->velocity[i];
            if (i < msg->effort.size())   motor_states_[id].effort   = msg->effort[i];
        }
    }

    void update_and_publish() {
        const rclcpp::Time t = now();
        double dt = (t - last_update_time_).seconds();
        if (dt <= 0.0) dt = 0.02;
        last_update_time_ = t;

        // --- Arm actuator integration ---
        step_actuator(ARM_ID, ARM_CFG, arm_percent_, dt);

        // --- Bucket actuator integration ---
        step_actuator(BUCKET_ID, BUCKET_CFG, bucket_percent_, dt);

        // Publish joint commands
        publish_joint_position_cmd(ARM_ID, cmd_pos_rad_[ARM_ID]);
        publish_joint_position_cmd(BUCKET_ID, cmd_pos_rad_[BUCKET_ID]);

        // Publish status at the same rate (or you can split timers if you want)
        publish_status();
    }

    void step_actuator(int id, const JointConfig& cfg, double percent, double dt) {
        // inches/sec limited by actuator speed
        const double v_in_s = percent * cfg.speed_in_per_s;

        // convert to rad/sec (linear mapping)
        const double w_rad_s = v_in_s * rad_per_in(cfg);

        // integrate commanded joint angle
        double next = cmd_pos_rad_[id] + w_rad_s * dt;

        // clamp to joint limits
        next = clampd(next, cfg.lower_rad, cfg.upper_rad);
        cmd_pos_rad_[id] = next;
    }

    void publish_status() {
        for (auto const& [id, pub] : status_publishers_) {
            messages::msg::TalonStatus status;
            status.device_id = id;

            double output_percent = 0.0;
            if (id == ARM_ID) output_percent = arm_percent_;
            if (id == BUCKET_ID) output_percent = bucket_percent_;
            status.output_percent = static_cast<float>(output_percent);

            const double pos = motor_states_.count(id) ? motor_states_[id].position : cmd_pos_rad_[id];
            const double vel = motor_states_.count(id) ? motor_states_[id].velocity : 0.0;

            if (id == ARM_ID) {
                status.sensor_position = pot_from_angle(pos, ARM_CFG);   // 0..1024, working 20..950
            }
            else if (id == BUCKET_ID) {
                status.sensor_position = pot_from_angle(pos, BUCKET_CFG);
            }
            else {
                status.sensor_position = 0;
            }

            status.sensor_velocity = 0;
            if(output_percent != 0.0)
                status.output_current = 1.0;
            else
                status.output_current = 0.0;
            status.bus_voltage = 16.0;
            status.temperature = 45.0;

            pub->publish(status);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TalonSimNode>());
    rclcpp::shutdown();
    return 0;
}
