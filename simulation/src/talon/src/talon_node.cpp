#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <messages/msg/talon_status.hpp> 
#include <map>
#include <string>
#include <cmath>

// Map internal IDs to Gazebo Controller Topics
const std::map<int, std::string> ID_TO_GAZEBO_TOPIC = {
    {14, "/talon_14_controller/commands"},
    {15, "/talon_15_controller/commands"},
    {16, "/talon_16_controller/commands"},
    {17, "/talon_17_controller/commands"}
};

// Map Gazebo Joint Names to IDs (for feedback)
const std::map<std::string, int> JOINT_NAME_TO_ID = {
    {"FL_Wheel_Joint", 14},
    {"FR_Wheel_Joint", 15},
    {"BL_Wheel_Joint", 16},
    {"BR_Wheel_Joint", 17}
};

double talon14Speed, talon15Speed, talon16Speed, talon17Speed;

class TalonSimNode : public rclcpp::Node {
public:
    TalonSimNode() : Node("talon_sim_node") {
        // 1. Setup Publishers to Gazebo & Status Publishers for Client
        for (auto const& [id, topic] : ID_TO_GAZEBO_TOPIC) {
            // Gazebo Command Publisher
            gazebo_publishers_[id] = this->create_publisher<std_msgs::msg::Float64MultiArray>(topic, 10);
            
            // Client Status Publisher (e.g. "talon_14_info" to match your code conventions if needed)
            // Your communication node listens to "talon_14_info" or "talon_10_status"? 
            // Based on communication_node.cpp, it listens to "talon_14_info" for Talon 1
            std::string status_topic = "talon_" + std::to_string(id) + "_info"; 
            status_publishers_[id] = this->create_publisher<messages::msg::TalonStatus>(status_topic, 10);
        }

        // 2. Subscribe to Drivetrain Node Outputs
        // We need individual subscriptions for each motor speed topic
        sub_10_ = this->create_subscription<std_msgs::msg::Float32>(
            "talon_14_speed", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) { send_command(14, msg->data); RCLCPP_INFO(this->get_logger(), "Talon 14 data: %f", msg->data); });
        
        sub_11_ = this->create_subscription<std_msgs::msg::Float32>(
            "talon_15_speed", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) { send_command(15, msg->data); RCLCPP_INFO(this->get_logger(), "Talon 15 data: %f", msg->data); });
        
        sub_12_ = this->create_subscription<std_msgs::msg::Float32>(
            "talon_16_speed", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) { send_command(16, msg->data); RCLCPP_INFO(this->get_logger(), "Talon 16 data: %f", msg->data); });
        
        sub_13_ = this->create_subscription<std_msgs::msg::Float32>(
            "talon_17_speed", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) { send_command(17, msg->data); RCLCPP_INFO(this->get_logger(), "Talon 17 data: %f", msg->data); });

        // 3. Subscribe to Gazebo Feedback
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, 
            std::bind(&TalonSimNode::joint_state_callback, this, std::placeholders::_1));

        // 4. Status Update Timer (100Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), 
            std::bind(&TalonSimNode::publish_status, this));
            
        RCLCPP_INFO(this->get_logger(), "Talon Simulation Bridge Started");
    }

private:
    struct SimMotorState {
        double position = 0.0;
        double velocity = 0.0;
        double effort = 0.0;
    };
    std::map<int, SimMotorState> motor_states_;

    std::map<int, rclcpp::Publisher<messages::msg::TalonStatus>::SharedPtr> status_publishers_;
    std::map<int, rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr> gazebo_publishers_;
    
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_10_, sub_11_, sub_12_, sub_13_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Helper to send command to Gazebo
    void send_command(int id, float speed_percent) {
        // Convert Percentage (-1.0 to 1.0) to Rad/s
        // Assuming max speed is roughly 10 rad/s (approx 100 RPM)
        double target_velocity = speed_percent * 10.0; 

        if(id == 14)
            talon14Speed = speed_percent;
        if(id == 15)
            talon15Speed = speed_percent;
        if(id == 16)
            talon16Speed = speed_percent;
        if(id == 17)
            talon17Speed = speed_percent;

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

    // Publish Status to Control Client (Mirrors the physical Talon node)
    void publish_status() {
        for (auto const& [id, pub] : status_publishers_) {
            messages::msg::TalonStatus status;
            status.device_id = id;

            if(id == 14)
                status.output_percent = talon14Speed;
            if(id == 15)
                status.output_percent = talon15Speed;
            if(id == 16)
                status.output_percent = talon16Speed;
            if(id == 17)
                status.output_percent = talon17Speed;
            
            // Conversions to match Real Hardware units
            status.sensor_position = 100; // Ticks
            status.sensor_velocity = 0; // Ticks/100ms
            status.output_current = 0.5; 
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