#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <messages/msg/falcon_status.hpp> 
#include <map>
#include <string>
#include <cmath>

// Map internal IDs to Gazebo Controller Topics
const std::map<int, std::string> ID_TO_GAZEBO_TOPIC = {
    {10, "/falcon_10_controller/commands"},
    {11, "/falcon_11_controller/commands"},
    {12, "/falcon_12_controller/commands"},
    {13, "/falcon_13_controller/commands"}
};

// Map Gazebo Joint Names to IDs (for feedback)
const std::map<std::string, int> JOINT_NAME_TO_ID = {
    {"FL_Wheel_Joint", 10},
    {"FR_Wheel_Joint", 11},
    {"BL_Wheel_Joint", 12},
    {"BR_Wheel_Joint", 13}
};

class FalconSimNode : public rclcpp::Node {
public:
    FalconSimNode() : Node("falcon_sim_node") {
        // 1. Setup Publishers to Gazebo & Status Publishers for Client
        for (auto const& [id, topic] : ID_TO_GAZEBO_TOPIC) {
            // Gazebo Command Publisher
            gazebo_publishers_[id] = this->create_publisher<std_msgs::msg::Float64MultiArray>(topic, 10);
            
            // Client Status Publisher (e.g. "talon_10_info" to match your code conventions if needed)
            // Your communication node listens to "talon_10_info" or "falcon_10_status"? 
            // Based on communication_node.cpp, it listens to "talon_10_info" for Falcon 1
            std::string status_topic = "talon_" + std::to_string(id) + "_info"; 
            status_publishers_[id] = this->create_publisher<messages::msg::FalconStatus>(status_topic, 10);
        }

        // 2. Subscribe to Drivetrain Node Outputs
        // We need individual subscriptions for each motor speed topic
        sub_10_ = this->create_subscription<std_msgs::msg::Float32>(
            "falcon_10_speed", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) { send_command(10, msg->data); RCLCPP_INFO(this->get_logger(), "Falcon 10 data: %f", msg->data); });
        
        sub_11_ = this->create_subscription<std_msgs::msg::Float32>(
            "falcon_11_speed", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) { send_command(11, msg->data); RCLCPP_INFO(this->get_logger(), "Falcon 11 data: %f", msg->data); });
        
        sub_12_ = this->create_subscription<std_msgs::msg::Float32>(
            "falcon_12_speed", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) { send_command(12, msg->data); RCLCPP_INFO(this->get_logger(), "Falcon 12 data: %f", msg->data); });
        
        sub_13_ = this->create_subscription<std_msgs::msg::Float32>(
            "falcon_13_speed", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) { send_command(13, msg->data); RCLCPP_INFO(this->get_logger(), "Falcon 13 data: %f", msg->data); });

        // 3. Subscribe to Gazebo Feedback
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, 
            std::bind(&FalconSimNode::joint_state_callback, this, std::placeholders::_1));

        // 4. Status Update Timer (100Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), 
            std::bind(&FalconSimNode::publish_status, this));
            
        RCLCPP_INFO(this->get_logger(), "Falcon Simulation Bridge Started");
    }

private:
    struct SimMotorState {
        double position = 0.0;
        double velocity = 0.0;
        double effort = 0.0;
    };
    std::map<int, SimMotorState> motor_states_;

    std::map<int, rclcpp::Publisher<messages::msg::FalconStatus>::SharedPtr> status_publishers_;
    std::map<int, rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr> gazebo_publishers_;
    
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_10_, sub_11_, sub_12_, sub_13_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Helper to send command to Gazebo
    void send_command(int id, float speed_percent) {
        // Convert Percentage (-1.0 to 1.0) to Rad/s
        // Assuming max speed is roughly 10 rad/s (approx 100 RPM)
        double target_velocity = speed_percent * 10.0; 

        std_msgs::msg::Float64MultiArray gazebo_cmd;
        gazebo_cmd.data.push_back(target_velocity);
        
        if (gazebo_publishers_.count(id)) {
            gazebo_publishers_[id]->publish(gazebo_cmd);
        }
    }

    // Read Sim Feedback from Gazebo
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            std::string name = msg->name[i];
            if (JOINT_NAME_TO_ID.count(name)) {
                int id = JOINT_NAME_TO_ID.at(name);
                motor_states_[id].position = msg->position[i];
                motor_states_[id].velocity = msg->velocity[i];
                if (msg->effort.size() > i) motor_states_[id].effort = msg->effort[i]; 
            }
        }
    }

    // Publish Status to Control Client (Mirrors the physical Falcon node)
    void publish_status() {
        for (auto const& [id, pub] : status_publishers_) {
            messages::msg::FalconStatus status;
            status.device_id = id;
            
            // Conversions to match Real Hardware units
            status.sensor_position = motor_states_[id].position * (2048.0 / (2.0 * M_PI)); // Ticks
            status.sensor_velocity = (motor_states_[id].velocity * (2048.0 / (2.0 * M_PI))) / 10.0; // Ticks/100ms
            status.output_current = std::abs(motor_states_[id].effort) * 5.0; 
            status.bus_voltage = 16.0; 
            status.temperature = 45.0;

            pub->publish(status);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FalconSimNode>());
    rclcpp::shutdown();
    return 0;
}