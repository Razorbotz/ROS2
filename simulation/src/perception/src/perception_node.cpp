#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>

// OpenCV integration
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// ros2 run perception perception_node
// ros2 run teleop_twist_keyboard teleop_twist_keyboard
// ros2 run rqt_image_view rqt_image_view
// export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:/home/team/SoftwareDevelopment/ROS2/simulation/install/sim/share/sim/models:/home/team/SoftwareDevelopment/ROS2/simulation/src/sim/models
// gazebo --verbose /home/team/SoftwareDevelopment/ROS2/simulation/src/sim/worlds/high_resolution/artemis/artemis_arena.world -s libgazebo_ros_factory.so -s libgazebo_ros_init.so
// ros2 launch launch/test_rig.launch.py

class LunarPerceptionNode : public rclcpp::Node
{
public:
    LunarPerceptionNode() : Node("lunar_perception_node")
    {
        // Subscribe to BOTH the point cloud and the raw image
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/d415_sensor/points", 10,
            std::bind(&LunarPerceptionNode::pointcloud_callback, this, std::placeholders::_1));

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/d415_sensor/image_raw", 10,
            std::bind(&LunarPerceptionNode::image_callback, this, std::placeholders::_1));

        // Publisher for the drawn 2D image
        debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/perception/debug_image", 10);
    }

private:
    cv::Mat current_image_;

    // Simple pinhole projection based on your D415 URDF specs
    cv::Point project3DTo2D(float x, float y, float z) {
        if (z <= 0) return cv::Point(-1, -1); // Ignore points behind the camera
        
        // Focal length calculated from 1280w and 1.211 FOV
        double fx = 927.4; 
        double fy = 927.4;
        double cx = 640.0; // Image center X
        double cy = 360.0; // Image center Y

        int u = static_cast<int>((x / z) * fx + cx);
        int v = static_cast<int>((y / z) * fy + cy);
        return cv::Point(u, v);
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Store the latest image for drawing
        try {
            current_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (current_image_.empty()) return; // Wait until we have an image stream

        // 1. Convert to PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);

        // --- Step A & B: Voxel Grid and RANSAC (from previous implementation) ---
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(0.05f, 0.05f, 0.05f);
        vg.filter(*cloud_filtered);

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(100);
        seg.setDistanceThreshold(0.05);
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obstacles(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*cloud_obstacles);

        // --- Step C: Euclidean Clustering (Group the rocks) ---
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        if (cloud_obstacles->points.empty()) return;
        tree->setInputCloud(cloud_obstacles);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.15); // Points within 15cm belong to the same rock
        ec.setMinClusterSize(10);     // Ignore tiny noise
        ec.setMaxClusterSize(25000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_obstacles);
        ec.extract(cluster_indices);

        // Clone the image so we can draw on it
        cv::Mat display_image = current_image_.clone();

        // --- Step D: Find 3D Bounding Boxes and Draw 2D Boxes ---
        for (const auto& cluster : cluster_indices)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
            for (const auto& idx : cluster.indices) {
                cloud_cluster->push_back((*cloud_obstacles)[idx]);
            }

            // Find the 3D extents of this cluster
            pcl::PointXYZ min_pt, max_pt;
            pcl::getMinMax3D(*cloud_cluster, min_pt, max_pt);

            // Project the 3D min/max boundaries into the 2D image
            // We use min/max X and Y at the front face (min Z) for a rough 2D box
            cv::Point top_left = project3DTo2D(min_pt.x, min_pt.y, min_pt.z);
            cv::Point bottom_right = project3DTo2D(max_pt.x, max_pt.y, min_pt.z);

            // Draw a green rectangle on the image
            if (top_left.x != -1 && bottom_right.x != -1) {
                cv::rectangle(display_image, top_left, bottom_right, cv::Scalar(0, 255, 0), 2);
                cv::putText(display_image, "Obstacle", top_left, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
            }
        }

        // Publish the drawn image
        sensor_msgs::msg::Image::SharedPtr out_msg = cv_bridge::CvImage(msg->header, "bgr8", display_image).toImageMsg();
        debug_image_pub_->publish(*out_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LunarPerceptionNode>());
    rclcpp::shutdown();
    return 0;
}