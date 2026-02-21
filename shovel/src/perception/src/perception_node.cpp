#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/detection3_d.hpp>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

class LunarPerceptionNode : public rclcpp::Node
{
public:
    LunarPerceptionNode() : Node("lunar_perception_node")
    {
        // 1. Subscribe to the RealSense Point Cloud
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/points", 10,
            std::bind(&LunarPerceptionNode::pointcloud_callback, this, std::placeholders::_1));

        // 2. Publisher for the evaluated Ground Truth comparison
        publisher_ = this->create_publisher<vision_msgs::msg::Detection3DArray>(
            "/perception/detections", 10);
            
        RCLCPP_INFO(this->get_logger(), "Lunar Perception Node Initialized.");
    }

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 1. Convert ROS PointCloud2 to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);

        // --- ALGORITHM START --- //

        // TODO: Step A - Voxel Grid Downsampling (Optimize performance)
        
        // TODO: Step B - RANSAC Planar Segmentation (Find the lunar floor)
        
        // TODO: Step C - Euclidean Cluster Extraction (Group rock points together)
        
        // TODO: Step D - Bounding Box Calculation (Find center X,Y,Z and sizes)

        // --- ALGORITHM END --- //

        // 2. Prepare the Output Message
        vision_msgs::msg::Detection3DArray detection_msg;
        detection_msg.header = msg->header; // Keep the same timestamp and optical frame

        // Example of adding a dummy detection to the array (You will loop through your clusters here)
        /*
        vision_msgs::msg::Detection3D detection;
        detection.bbox.center.position.x = calculated_x;
        detection.bbox.center.position.y = calculated_y;
        detection.bbox.center.position.z = calculated_z;
        detection.bbox.size.x = size_x;
        detection.bbox.size.y = size_y;
        detection.bbox.size.z = size_z;
        detection_msg.detections.push_back(detection);
        */

        // 3. Publish
        // publisher_->publish(detection_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LunarPerceptionNode>());
    rclcpp::shutdown();
    return 0;
}