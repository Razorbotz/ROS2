#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <librealsense2/rs.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <vector>

bool is_inside(const cv::Rect& inner, const cv::Rect& outer) {
    return (outer.x <= inner.x && outer.y <= inner.y &&
            outer.x + outer.width >= inner.x + inner.width &&
            outer.y + outer.height >= inner.y + inner.height);
}

float distance_to_plane(float x, float y, float z, const pcl::ModelCoefficients::Ptr& coeffs) {
    float A = coeffs->values[0];
    float B = coeffs->values[1];
    float C = coeffs->values[2];
    float D = coeffs->values[3];
    return std::abs(A * x + B * y + C * z + D) / std::sqrt(A * A + B * B + C * C);
}

void detect_obstacles(
    rs2::pipeline& pipe,
    const rs2_intrinsics& intrin,
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub,
    rclcpp::Clock::SharedPtr clock)
{
    float height_threshold = 0.10f;
    int min_area = 900;
    int kernelSize = 25;

    rs2::frameset frames = pipe.wait_for_frames();
    rs2::depth_frame depth = frames.get_depth_frame();
    int width = depth.get_width();
    int height = depth.get_height();

    rs2::pointcloud pc;
    rs2::points points = pc.calculate(depth);
    const rs2::vertex* verts = points.get_vertices();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->points.reserve(points.size());
    for (int i = 0; i < points.size(); ++i) {
        if (!std::isnan(verts[i].z) && verts[i].z > 0.0f) {
            cloud->points.emplace_back(verts[i].x, verts[i].y, verts[i].z);
        }
    }

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coeffs);
    if (inliers->indices.empty()) return;

    cv::Mat binary_mask(height, width, CV_8U, cv::Scalar(0));

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            float dist = depth.get_distance(x, y);
            if (dist <= 0.0f || dist > 5.0f) continue;

            float pixel[2] = { (float)x, (float)y };
            float point[3];
            rs2_deproject_pixel_to_point(point, &intrin, pixel, dist);

            float d_plane = distance_to_plane(point[0], point[1], point[2], coeffs);
            if (d_plane > height_threshold) {
                binary_mask.at<uchar>(y, x) = 255;
            }
        }
    }

    cv::Mat closed_mask;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernelSize, kernelSize));
    cv::morphologyEx(binary_mask, closed_mask, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(closed_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::Rect> boxes;
    for (const auto& contour : contours) {
        if (cv::contourArea(contour) >= min_area)
            boxes.push_back(cv::boundingRect(contour));
    }

    for (size_t i = 0; i < boxes.size(); ++i) {
        bool nested = false;
        for (size_t j = 0; j < boxes.size(); ++j) {
            if (i != j && is_inside(boxes[i], boxes[j])) {
                nested = true;
                break;
            }
        }
        if (nested) continue;

        int cx = boxes[i].x + boxes[i].width / 2;
        int cy = boxes[i].y + boxes[i].height / 2;

        float dist = depth.get_distance(cx, cy);
        if (dist > 0.0f && dist < 5.0f) {
            float pixel[2] = { (float)cx, (float)cy };
            float point[3];
            rs2_deproject_pixel_to_point(point, &intrin, pixel, dist);

            geometry_msgs::msg::PointStamped msg;
            msg.header.stamp = clock->now();
            msg.header.frame_id = "camera_link";
            msg.point.x = point[0];
            msg.point.y = point[1];
            msg.point.z = point[2];
            pub->publish(msg);
        }
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("realsense_node");

    auto pub = node->create_publisher<geometry_msgs::msg::PointStamped>("obstacle_positions", 10);
    auto clock = node->get_clock();

    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    pipe.start(cfg);

    auto depth_profile = pipe.wait_for_frames().get_depth_frame().get_profile();
    rs2::video_stream_profile vsp = depth_profile.as<rs2::video_stream_profile>();
    rs2_intrinsics intrin = vsp.get_intrinsics();

    auto timer = node->create_wall_timer(
        std::chrono::milliseconds(100),
        [&, intrin]() mutable {
            detect_obstacles(pipe, intrin, pub, clock);
        });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

