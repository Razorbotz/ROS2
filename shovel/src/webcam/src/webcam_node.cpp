#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>

using namespace std::chrono_literals;

class WebcamNode : public rclcpp::Node {
public:
    WebcamNode() : Node("webcam") {
        RCLCPP_INFO(this->get_logger(), "Starting webcam node...");

        // Match the topic name of the ZED node so downstream nodes don't break
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("zed_image", 10);

        // Open the default camera (index 0)
        // If you have multiple webcams, you may need to change this to 1, 2, etc.
        cap_.open(0);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open webcam! Check connections and permissions.");
            return;
        }

        // Try to set standard resolution (optional, adjust to your needs)
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        cap_.set(cv::CAP_PROP_FPS, 30);

        // Timer to pull frames at ~30Hz
        timer_ = this->create_wall_timer(
            33ms, std::bind(&WebcamNode::timer_callback, this));
    }

    ~WebcamNode() {
        if (cap_.isOpened()) {
            cap_.release();
        }
    }

private:
    void timer_callback() {
        cv::Mat frame;
        cv::Mat rgb_frame;

        if (cap_.read(frame)) {
            // OpenCV captures in BGR by default. 
            // The ZED node published "rgb8", so we convert it here to perfectly match the original stream format.
            cv::cvtColor(frame, rgb_frame, cv::COLOR_BGR2RGB);

            std_msgs::msg::Header hdr;
            hdr.stamp = this->get_clock()->now();
            hdr.frame_id = "webcam_link"; // Arbitrary frame ID since we aren't doing TF tracking

            // Convert OpenCV Mat to ROS 2 Image message
            sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(hdr, "rgb8", rgb_frame).toImageMsg();
            
            image_publisher_->publish(*msg);
        } else {
            RCLCPP_WARN(this->get_logger(), "Dropped frame - could not read from webcam.");
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WebcamNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}