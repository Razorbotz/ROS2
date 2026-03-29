#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

// ADDED: TF2 Includes for coordinate transformation
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>

class LunarPerceptionNode : public rclcpp::Node
{
public:
    LunarPerceptionNode() : Node("lunar_perception_node")
    {
        // Initialize TF2 Buffer and Listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/points", 10,
            std::bind(&LunarPerceptionNode::pointcloud_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<vision_msgs::msg::Detection3DArray>("/perception/detections", 10);
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/perception/markers", 10);
            
        RCLCPP_INFO(this->get_logger(), "Lunar Perception Node Initialized with TF2.");
    }

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 1. Transform the PointCloud to base_link (Gravity-aligned)
        geometry_msgs::msg::TransformStamped transform;
        try {
            // Wait up to 0.1 seconds for the transform tree to be ready
            transform = tf_buffer_->lookupTransform("base_link", msg->header.frame_id, 
                                                    msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "TF Error: %s", ex.what());
            return;
        }

        sensor_msgs::msg::PointCloud2 transformed_msg;
        tf2::doTransform(*msg, transformed_msg, transform);

        // 2. Convert to PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(transformed_msg, *cloud);

        if (cloud->empty()) return;

        // STEP A: Voxel Grid Downsampling
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(0.05f, 0.05f, 0.05f); 
        vg.filter(*cloud_filtered);

        // STEP A.5: PassThrough Filter (In base_link, X is Forward, Y is Side)
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cropped(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud_filtered);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(0.2, 3.0); // Look forward up to 3 meters
        pass.filter(*cloud_cropped);
        
        pass.setInputCloud(cloud_cropped);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(-1.5, 1.5); // 3 meters wide
        pass.filter(*cloud_cropped);

        // STEP B: RANSAC Planar Segmentation
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(100);
        seg.setDistanceThreshold(0.08); // 8cm tolerance for floor
        seg.setInputCloud(cloud_cropped);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty()) return;

        // Extract Obstacles
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obstacles(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_cropped);
        extract.setIndices(inliers);
        extract.setNegative(true); 
        extract.filter(*cloud_obstacles);

        // STEP C: Euclidean Cluster Extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud_obstacles);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.08); // 8cm gap between objects
        ec.setMinClusterSize(15);     
        ec.setMaxClusterSize(1500);  
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_obstacles);
        ec.extract(cluster_indices);

        // Prepare Output Messages (Using transformed base_link header)
        vision_msgs::msg::Detection3DArray detection_msg;
        detection_msg.header = transformed_msg.header; 
        
        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker delete_all;
        delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(delete_all);

        int marker_id = 0;

        // STEP D: Bounding Box Calculation
        for (const auto& cluster : cluster_indices)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>());
            for (const auto& idx : cluster.indices) {
                cloud_cluster->push_back((*cloud_obstacles)[idx]);
            }

            pcl::PointXYZ min_pt, max_pt;
            pcl::getMinMax3D(*cloud_cluster, min_pt, max_pt);

            min_pt.z = 0.0;
            vision_msgs::msg::Detection3D detection;
            detection.bbox.center.position.x = (min_pt.x + max_pt.x) / 2.0;
            detection.bbox.center.position.y = (min_pt.y + max_pt.y) / 2.0;
            detection.bbox.center.position.z = (min_pt.z + max_pt.z) / 2.0;
            detection.bbox.center.orientation.w = 1.0; 
            detection.bbox.size.x = max_pt.x - min_pt.x;
            detection.bbox.size.y = max_pt.y - min_pt.y;
            detection.bbox.size.z = max_pt.z - min_pt.z;
            detection_msg.detections.push_back(detection);

            visualization_msgs::msg::Marker marker;
            marker.header = transformed_msg.header; // Now securely in base_link
            marker.ns = "rock_clusters";
            marker.id = marker_id++;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose = detection.bbox.center;
            marker.scale = detection.bbox.size;
            marker.color.r = 0.0f; marker.color.g = 1.0f; marker.color.b = 0.0f; marker.color.a = 0.5f;
            marker.lifetime = rclcpp::Duration::from_seconds(0.5); 
            marker_array.markers.push_back(marker);
        }

        publisher_->publish(detection_msg);
        marker_publisher_->publish(marker_array);
    }

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LunarPerceptionNode>());
    rclcpp::shutdown();
    return 0;
}