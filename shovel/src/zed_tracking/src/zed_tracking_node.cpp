#include <rclcpp/rclcpp.hpp>
#include <sl/Camera.hpp>
#include "aruco.hpp"
#include <opencv2/opencv.hpp>
#include "messages/msg/zed_position.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

#include <messages/msg/key_state.hpp>
#include <cstdio>
#include <unistd.h> 
#include <fcntl.h>
#include <cstdlib>
#include "utils/utils.hpp"

rclcpp::Node::SharedPtr nodeHandle;

// Constants
const std::string POSITION_FILE = "/tmp/position.txt";
const std::string SHUTDOWN_MARKER_FILE = "/tmp/clean_shutdown.txt";
std::string AREA_MAP =  "AreaMap.area";

sl::Camera zed;

int killKey = 0;
bool printData = false;

void write_shutdown_marker() {
    FILE* fp = fopen(SHUTDOWN_MARKER_FILE.c_str(), "w");
    if (fp) {
        fputs("clean", fp);
        fflush(fp);  // Flush C library buffers
        fsync(fileno(fp));  // Flush OS file system buffers
        fclose(fp);
    }
    else {
        RCLCPP_ERROR(nodeHandle->get_logger(), "Failed to write shutdown marker.");
    }
}

void signal_handler(int signum) {
    RCLCPP_INFO(nodeHandle->get_logger(), "Signal received. Performing clean shutdown.");
    write_shutdown_marker();
    rclcpp::shutdown();
}

inline float deg2rad(float degrees) {
    return degrees * static_cast<float>(M_PI) / 180.0f;
}


void check_for_crash() {
    std::ifstream marker_file(SHUTDOWN_MARKER_FILE);
    if (!marker_file.good()) {
        RCLCPP_WARN(nodeHandle->get_logger(), "Previous crash detected. Attempting recovery...");

        std::ifstream pos_file(POSITION_FILE);
        if (pos_file.is_open()) {
            std::string line;
            if (std::getline(pos_file, line)) {
                RCLCPP_INFO(nodeHandle->get_logger(), "Recovered position: %s", line.c_str());
                
                std::istringstream ss(line);
                std::string token;
                std::vector<float> values;

                while (std::getline(ss, token, ',')) {
                    try {
                        values.push_back(std::stof(token));
                    }
                    catch (const std::exception& e) {
                        RCLCPP_ERROR(nodeHandle->get_logger(), "Error parsing float from: '%s'", token.c_str());
                    }
                }

                if (values.size() == 6) {
                    float tx = values[0], ty = values[1], tz = values[2];
                    float roll = values[3], pitch = values[4], yaw = values[5];

                    RCLCPP_INFO(nodeHandle->get_logger(),
                                "Recovered: Pos(%.2f, %.2f, %.2f), Rot(%.2f°, %.2f°, %.2f°)",
                                tx, ty, tz, roll, pitch, yaw);

                    float roll_rad = deg2rad(roll);
                    float pitch_rad = deg2rad(pitch);
                    float yaw_rad = deg2rad(yaw);

                    // Build ZED Transform directly from translation and Euler angles
                    sl::Transform init_pose;
                    init_pose.setTranslation(sl::Translation(tx, ty, tz));
                    sl::float3 rpy(roll_rad, pitch_rad, yaw_rad);
                    
                    sl::Rotation rot;
                    rot.setEulerAngles(rpy, true); // true = radians
                    init_pose.setOrientation(sl::Orientation(rot));

                    // Reset ZED positional tracking with this transform
                    sl::ERROR_CODE err = zed.resetPositionalTracking(init_pose);
                    if (err != sl::ERROR_CODE::SUCCESS) {
                        std::cerr << "Failed to reset positional tracking: " << sl::toString(err) << std::endl;
                    } else {
                        std::cout << "ZED position successfully reset to saved pose." << std::endl;
                    }
                }
                else {
                    RCLCPP_ERROR(nodeHandle->get_logger(), "Expected 6 values, got %zu", values.size());
                }
            }
            pos_file.close();
        }
        else {
            RCLCPP_WARN(nodeHandle->get_logger(), "No position file found.");
        }
    }
    else {
        marker_file.close();
        std::remove(SHUTDOWN_MARKER_FILE.c_str());
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc,argv);
    nodeHandle = rclcpp::Node::make_shared("zed_tracking");

    RCLCPP_INFO(nodeHandle->get_logger(),"Starting zed_tracking");

    std::string resolution = utils::getParameter<std::string>(nodeHandle, "resolution", "VGA");
    double xOffset = utils::getParameter<double>(nodeHandle, "xOffset", 0.0);
	killKey = utils::getParameter<int>(nodeHandle, "kill_key", 0);
    printData = utils::getParameter<bool>(nodeHandle, "print_data", false);

    auto zedPositionPublisher = nodeHandle->create_publisher<messages::msg::ZedPosition>("zed_position", 1);
    image_transport::ImageTransport it(nodeHandle);
    image_transport::Publisher zedImagePublisher = it.advertise("zed_image", 1);

    sl::InitParameters init_params;
    init_params.camera_resolution = sl::RESOLUTION::HD720;
    init_params.camera_fps = 30; 
    init_params.coordinate_units = sl::UNIT::METER;
//    init_params.coordinate_system = sl::COORDINATE_SYSTEM::IMAGE;
//    init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
//    init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP;
    init_params.coordinate_system = sl::COORDINATE_SYSTEM::LEFT_HANDED_Y_UP;
//    init_params.coordinate_system = sl::COORDINATE_SYSTEM::LEFT_HANDED_Z_UP;
//    init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
    init_params.sensors_required = true;
    init_params.depth_mode = sl::DEPTH_MODE::NEURAL;

    init_params.svo_real_time_mode = false;
    init_params.camera_image_flip = sl::FLIP_MODE::AUTO;

    // Open the camera
    auto err = zed.open(init_params);
    if (err != sl::ERROR_CODE::SUCCESS) {
	    std::cout << "Error, unable to open ZED camera: " << err << "\n";
        zed.close();
        return 1; // Quit if an error occurred
    }

    auto cameraInfo = zed.getCameraInformation().camera_configuration;
    sl::Resolution image_size = cameraInfo.resolution;
    sl::Mat image_zed(image_size, sl::MAT_TYPE::U8_C4);
    cv::Mat image_ocv = cv::Mat(image_zed.getHeight(), image_zed.getWidth(), CV_8UC4, image_zed.getPtr<sl::uchar1>(sl::MEM::CPU), image_zed.getStepBytes(sl::MEM::CPU));
    cv::Mat image_ocv_rgb;

    auto calibInfo = cameraInfo.calibration_parameters.left_cam;
    cv::Matx33d camera_matrix = cv::Matx33d::eye();
    camera_matrix(0, 0) = calibInfo.fx;
    camera_matrix(1, 1) = calibInfo.fy;
    camera_matrix(0, 2) = calibInfo.cx;
    camera_matrix(1, 2) = calibInfo.cy;
    cv::Matx<float, 4, 1> dist_coeffs = cv::Vec4f::zeros();

    float actual_marker_size_meters = 0.165f; // real marker size in meters
   // float actual_marker_size_meters = 0.16f; //fake marker size in meters
    auto dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_100);

    // Tracking setup
    sl::PositionalTrackingParameters tracking_params;
    tracking_params.enable_imu_fusion = true;
    tracking_params.enable_area_memory = true;
    tracking_params.set_gravity_as_origin = true; 
    tracking_params.mode = sl::POSITIONAL_TRACKING_MODE::GEN_3;
    
    auto returned_state = zed.enablePositionalTracking(tracking_params);
    if (returned_state != sl::ERROR_CODE::SUCCESS) {
        zed.close();
        return EXIT_FAILURE;
    }

    // Runtime params from main.cpp to reduce noise
    sl::RuntimeParameters runtime_params;
    runtime_params.confidence_threshold = 30;

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
    check_for_crash();

    // Initialization Variables
    bool origin_locked = false;
    int valid_frames_collected = 0;
    const int FRAMES_TO_AVERAGE = 10;
    sl::float3 sum_t(0,0,0);
    sl::float3 sum_euler(0,0,0);

    // Define: Marker in World Frame
    sl::Transform T_marker_to_world;
    T_marker_to_world.setIdentity();
    T_marker_to_world.setTranslation(sl::Translation(xOffset, 0.0f, 0.0f)); 
    
    // Rotate 180 deg around Y so World +Z points outward from the marker surface
    sl::Rotation rot180;
    rot180.setEulerAngles(sl::float3(0, M_PI, 0), true); 
    T_marker_to_world.setOrientation(sl::Orientation(rot180));

    sl::Transform cv_to_lhyu;
    cv_to_lhyu.setIdentity();
    cv_to_lhyu.r11 = -1.0f;

    sl::Pose zedPose;
    messages::msg::ZedPosition zedPosition;
    std_msgs::msg::Header hdr;
    int writeCounter = 1;

    rclcpp::Rate rate(30);
    while (rclcpp::ok()) {
        if (zed.grab(runtime_params) == sl::ERROR_CODE::SUCCESS) {
            zed.retrieveImage(image_zed, sl::VIEW::LEFT, sl::MEM::CPU, image_size);
            cv::cvtColor(image_ocv, image_ocv_rgb, cv::COLOR_BGRA2BGR);
            
            std::vector<cv::Vec3d> rvecs, tvecs;
            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners;

            aruco::detectMarkers(image_ocv_rgb, dictionary, corners, ids);
            zedPosition.aruco_visible = (ids.size() > 0);

            // Origin locking logic
            if (!origin_locked && ids.size() > 0) {
                cv::Mat grayImage;
                cv::cvtColor(image_ocv_rgb, grayImage, cv::COLOR_BGR2GRAY);
                cv::cornerSubPix(grayImage, corners[0], cv::Size(5, 5), cv::Size(-1, -1),
                                 cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));

                aruco::estimatePoseSingleMarkers(corners, actual_marker_size_meters, camera_matrix, dist_coeffs, rvecs, tvecs);

                // 1. T_cam_to_marker in CV Frame
                cv::Mat R_cv;
                cv::Rodrigues(rvecs[0], R_cv);
                sl::Matrix3f R_sl;
                for(int r=0; r<3; r++)
                    for(int c=0; c<3; c++)
                        R_sl(r,c) = R_cv.at<double>(r,c);

                sl::Transform T_cam_to_marker_cv;
                T_cam_to_marker_cv.setRotationMatrix(R_sl);
                T_cam_to_marker_cv.setTranslation(sl::Translation(tvecs[0](0), tvecs[0](1), tvecs[0](2)));

                // 2. Convert to ZED LHYU Frame
                sl::Transform T_cam_to_marker_zed = cv_to_lhyu * T_cam_to_marker_cv * cv_to_lhyu;

                // 3. Compute T_world_to_cam
                sl::Transform T_world_to_cam = T_marker_to_world * sl::Transform::inverse(T_cam_to_marker_zed);

                sum_t += T_world_to_cam.getTranslation();
                sum_euler += T_world_to_cam.getEulerAngles(true);
                valid_frames_collected++;

                if (valid_frames_collected >= FRAMES_TO_AVERAGE) {
                    sl::Transform avg_T;
                    avg_T.setTranslation(sum_t / static_cast<float>(FRAMES_TO_AVERAGE));
                    
                    sl::Rotation avg_r;
                    avg_r.setEulerAngles(sum_euler / static_cast<float>(FRAMES_TO_AVERAGE), true);
                    avg_T.setOrientation(sl::Orientation(avg_r));

                    zed.resetPositionalTracking(avg_T);
                    origin_locked = true;
                    RCLCPP_INFO(nodeHandle->get_logger(), "Origin successfully locked to ArUco Marker!");
                }
            }

            // Normal operation post-lock
            auto tracking_state = zed.getPosition(zedPose, sl::REFERENCE_FRAME::WORLD);
            
            if (tracking_state == sl::POSITIONAL_TRACKING_STATE::OK) {
                sl::Transform current_pose = zedPose.pose_data;

                zedPosition.x = current_pose.getTranslation().x;
                zedPosition.y = current_pose.getTranslation().y;
                zedPosition.z = current_pose.getTranslation().z;
                
                sl::Orientation q = current_pose.getOrientation();
                zedPosition.ox = q.ox;
                zedPosition.oy = q.oy;
                zedPosition.oz = q.oz;
                zedPosition.ow = q.ow;
                
                sl::float3 euler = current_pose.getEulerAngles(false);
                zedPosition.roll = euler.x;
                zedPosition.pitch = euler.y;
                zedPosition.yaw = euler.z;

                zedPosition.aruco_initialized = origin_locked;
                zedPositionPublisher->publish(zedPosition);

                if(printData) {
                    RCLCPP_INFO(nodeHandle->get_logger(), "ZED x: %.3f, y: %.3f, z: %.3f", zedPosition.x, zedPosition.y, zedPosition.z);
                }

                if(writeCounter % 10 == 0){
                    std::ostringstream oss;
                    oss << zedPosition.x << "," << zedPosition.y << "," << zedPosition.z << "," 
                        << euler.x << "," << euler.y << "," << euler.z;

                    std::string tmp_path = POSITION_FILE + ".tmp";
                    int fd = open(tmp_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
                    if (fd != -1) {
                        std::string data = oss.str();
                        write(fd, data.c_str(), data.size());
                        fsync(fd);
                        close(fd);
                        std::rename(tmp_path.c_str(), POSITION_FILE.c_str());
                    }
                }
                writeCounter++;
            }

            if(!image_ocv_rgb.empty()){
                sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(hdr, "rgb8", image_ocv_rgb).toImageMsg();
                zedImagePublisher.publish(msg);
            }

/*
            if(stopped_for_excavation){
                RCLCPP_INFO(nodeHandle->get_logger(), "Before writing area map");
                // Save area map when the robot is stopped for excavation
                zed.saveAreaMap(sl::String(AREA_MAP.c_str()));
                if (std::rename(TEMP_MAP.c_str(), AREA_MAP.c_str()) != 0) {
                    RCLCPP_ERROR(nodeHandle->get_logger(), "Failed to rename temp position file.");
                }
            }
*/

        }
        rate.sleep();
    }
    
    zed.close();
    write_shutdown_marker();
    rclcpp::shutdown();
    return 0;
}