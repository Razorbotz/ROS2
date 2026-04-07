#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <thread>

class WebcamNode : public rclcpp::Node {
public:
    WebcamNode() : Node("webcam") {
        RCLCPP_INFO(this->get_logger(), "Starting webcam node...");

        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("zed_image", 10);

        // Open the default camera using the V4L2 backend
        cap_.open(0, cv::CAP_V4L2);
        
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open webcam!");
            return;
        }

        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

        // Spin up a dedicated background thread to read frames as fast as the camera provides them
        capture_thread_ = std::thread(&WebcamNode::capture_loop, this);
    }

    ~WebcamNode() {
        // Wait for the thread to finish cleanly on shutdown
        if (capture_thread_.joinable()) {
            capture_thread_.join();
        }
        if (cap_.isOpened()) {
            cap_.release();
        }
    }

private:
    void capture_loop() {
        cv::Mat frame;
        cv::Mat rgb_frame;

        // Loop continuously as long as the ROS node is alive
        while (rclcpp::ok()) {
            // cap_.read() is a blocking call, it will naturally pace the loop to the camera's true FPS
            if (cap_.read(frame) && !frame.empty()) {
                cv::cvtColor(frame, rgb_frame, cv::COLOR_BGR2RGB);

                std_msgs::msg::Header hdr;
                hdr.stamp = this->get_clock()->now();
                hdr.frame_id = "webcam_link"; 

                sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(hdr, "rgb8", rgb_frame).toImageMsg();
                image_publisher_->publish(*msg);
            } else {
                // Use a throttled warning so we don't spam the terminal if it gets unplugged
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                    "Dropped frame - could not read from webcam.");
            }
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    cv::VideoCapture cap_;
    std::thread capture_thread_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WebcamNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}