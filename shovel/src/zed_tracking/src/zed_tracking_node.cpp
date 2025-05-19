
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

#define ROW_COUNT 10
rclcpp::Node::SharedPtr nodeHandle;

// Create a ZED camera object
sl::Camera zed;

int killKey = 0;
//using namespace sl;
//using namespace std;

/** @file
 * @brief Node handling Zed camera
 * 
 * This node does not receive any information from other nodes, therefore, it does not subscribe to any node. Current purpose is to utilize the "ArUco Positional Tracking sample" with the Zed camera and publishes topics relating to it. Node contains no other functions, only main.
 * \see aruco.cpp
 *  
 * The topics that are being published are as follows:
 * \li \b zedPosition
 * "zedPosition" contains the variables x, y, z, ox, oy, oz, ow, and aruco_visible.
 * 
 * The variables x, y, and z are the translation vectors. 
 * To learn more about the translation vectors, see https://en.wikipedia.org/wiki/Translation_(geometry)
 * 
 * The variables ox, oy, oz, and ow are the orientation vectors. The orientation data is also known as "quaternion" data. These vectors help with calculating three-dimensional rotations.
 * To learn more about quaternion, see https://en.wikipedia.org/wiki/Quaternion
 * 
 * The variable "aruco_visible" tells whether or not that at least one marker is detected.
 * 
 * \see ZedPosition.msg
 * 
 * Zed camera currently using WVGA mode which has a FOV of 56(V) and 87(H).
 * 
 * Nodes that subscribe to the published Zed topics include the logic and autonomy node.
 * \see logic_node.cpp
 * 
 * \see autonomy_node.cpp
 * 
 * 
 * */


 /** @brief Function to get the value of the specified parameter
 * 
 * Function that takes a string as a parameter containing the
 * name of the parameter that is being parsed from the launch
 * file and the initial value of the parameter as inputs, then
 * gets the parameter, casts it as the desired type, displays 
 * the value of the parameter on the command line and the log 
 * file, then returns the parsed value of the parameter.
 * @param parametername String of the name of the parameter
 * @param initialValue Initial value of the parameter
 * @return value Value of the parameter
 * */
template <typename T>
T getParameter(std::string parameterName, T initialValue){
	nodeHandle->declare_parameter<T>(parameterName, initialValue);
	rclcpp::Parameter param = nodeHandle->get_parameter(parameterName);
	T value = param.template get_value<T>();
	std::cout << parameterName << ": " << value << std::endl;
	RCLCPP_INFO(nodeHandle->get_logger(), param.value_to_string().c_str());
	return value;
}

template <typename T>
T getParameter(const std::string& parameterName, const char* initialValue){
	return getParameter<T>(parameterName, std::string(initialValue));
}


bool isTagValidForReset(const std::vector<cv::Point2f> &corners, const cv::Size &image_size, float ratio = 0.05) {
  float image_area = image_size.width * image_size.height;

  double side1 = cv::norm(corners[1] - corners[0]);
  double side2 = cv::norm(corners[2] -corners[1]);

  float tag_area = side1 * side2;

  auto size_ratio = tag_area / image_area;

  return size_ratio > ratio;
}

const std::string POSITION_FILE = "/tmp/position.txt";
const std::string SHUTDOWN_MARKER_FILE = "/tmp/clean_shutdown.txt";
std::string AREA_MAP =  "AreaMap.area";
std::string TEMP_MAP = "TempMap.area";

void write_shutdown_marker() {
    FILE* fp = fopen(SHUTDOWN_MARKER_FILE.c_str(), "w");
    if (fp) {
        fputs("clean", fp);
        fflush(fp);  // Flush C library buffers
        fsync(fileno(fp));  // Flush OS file system buffers
        fclose(fp);
    } else {
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
                    init_pose.setEulerAngles(rpy, true); // bool radian = true

                    // Reset ZED positional tracking with this transform
                    sl::ERROR_CODE err = zed.resetPositionalTracking(init_pose);
                    if (err != sl::ERROR_CODE::SUCCESS) {
                        std::cerr << "Failed to reset positional tracking: " << sl::toString(err) << std::endl;
                    } else {
                        std::cout << "ZED position successfully reset to saved pose." << std::endl;
                    }

                    sl::PositionalTrackingParameters tracking_params;
                    tracking_params.enable_imu_fusion = true;
                    tracking_params.enable_area_memory = true;
                    tracking_params.enable_pose_smoothing = true;
                    tracking_params.mode = sl::POSITIONAL_TRACKING_MODE::GEN_2;
                    tracking_params.area_file_path = "AreaMap.area";
                    auto returned_state = zed.enablePositionalTracking(tracking_params);
                    if (returned_state != sl::ERROR_CODE::SUCCESS) {
                        zed.close();
                        return;
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


void keyCallback(const messages::msg::KeyState::SharedPtr keyState){
    if(keyState->key==killKey && keyState->state==1){
        RCLCPP_INFO(nodeHandle->get_logger(), "Kill key pressed");
        std::exit(0);
        return;
    }
}


int main(int argc, char **argv) {
    rclcpp::init(argc,argv);
    nodeHandle = rclcpp::Node::make_shared("zed_tracking");


    RCLCPP_INFO(nodeHandle->get_logger(),"Starting zed_tracking");

    std::string resolution = getParameter<std::string>("resolution", "VGA");
    double xOffset = getParameter<double>("xOffset", 0.0);
	killKey = getParameter<int>("kill_key", 0);

    messages::msg::ZedPosition zedPosition;
    auto zedPositionPublisher=nodeHandle->create_publisher<messages::msg::ZedPosition>("zed_position",1);

    image_transport::ImageTransport it(nodeHandle);
    image_transport::Publisher zedImagePublisher = it.advertise("zed_image", 1);
    std_msgs::msg::Header hdr;
    sensor_msgs::msg::Image::SharedPtr msg;

    // Set configuration parameters
    sl::InitParameters init_params;

    /*
    Camera Resolution Options (https://www.stereolabs.com/docs/api/group__Video__group.html#gabd0374c748530a64a72872c43b2cc828)
    HD2K 	
    -2208*1242 (x2),
    -available framerates: 15 fps
    -FOV: 47(V), 76(H)

    HD1080 	
    -1920*1080 (x2)
    -available framerates: 15, 30 fps
   -FOV: 42(V), 69(H)

    HD720 	
    -1280*720 (x2)
    -available framerates: 15, 30, 60 fps.
   -FOV: 54(V), 85(H)

    VGA	
    -672*376 (x2)
    -available framerates: 15, 30, 60, 100 fps.   
    -FOV: 56(V), 87(H)
    */
    if(resolution == "VGA"){
        init_params.camera_resolution = sl::RESOLUTION::HD720;
        init_params.camera_fps = 30;    
    }
    else if(resolution == "HD720"){
        init_params.camera_resolution = sl::RESOLUTION::HD720;
        init_params.camera_fps = 30; 
    }
    else if(resolution == "HD1080"){
        init_params.camera_resolution = sl::RESOLUTION::HD1080;
        init_params.camera_fps = 30; 
    }
    else if(resolution == "HD2K"){
        init_params.camera_resolution = sl::RESOLUTION::HD2K;
        init_params.camera_fps = 15; 
    }
    else{
        init_params.camera_resolution = sl::RESOLUTION::HD720;
        init_params.camera_fps = 30; 
    }
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
    sl::Mat depth, point_cloud;

    auto calibInfo = cameraInfo.calibration_parameters.left_cam;
    cv::Matx33d camera_matrix = cv::Matx33d::eye();
    camera_matrix(0, 0) = calibInfo.fx;
    camera_matrix(1, 1) = calibInfo.fy;
    camera_matrix(0, 2) = calibInfo.cx;
    camera_matrix(1, 2) = calibInfo.cy;

    cv::Matx<float, 4, 1> dist_coeffs = cv::Vec4f::zeros();

    float actual_marker_size_meters = 0.165f; // real marker size in meters
   // float actual_marker_size_meters = 0.16f; //fake marker size in meters
    auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100);

    std::cout << "Make sure the ArUco marker is a 6x6 (100), measuring " << actual_marker_size_meters * 1000 << " mm" << std::endl;

    sl::Transform pose;
    sl::Pose zedPose;
    std::vector<cv::Vec3d> rvecs, tvecs;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    std::string zed_position_txt;
    std::string zed_rotation_txt;
    std::string aruco_position_txt;
    sl::float3 angles;

    float auto_reset_aruco_screen_ratio = 0.01;

    bool can_reset = false;
    bool has_reset = false;

    sl::PositionalTrackingParameters tracking_params;
    tracking_params.enable_imu_fusion = true;
    tracking_params.enable_area_memory = true;
    tracking_params.enable_pose_smoothing = true;
    tracking_params.mode = sl::POSITIONAL_TRACKING_MODE::GEN_2;
    auto returned_state = zed.enablePositionalTracking(tracking_params);
    if (returned_state != sl::ERROR_CODE::SUCCESS) {
        zed.close();
        return EXIT_FAILURE;
    }

    sl::SensorsData sensors_data;
    sl::SensorsData::IMUData imu_data;

    sl::Transform ARUCO_TO_IMAGE_basis_change;
    ARUCO_TO_IMAGE_basis_change.r00 = -1;
    ARUCO_TO_IMAGE_basis_change.r11 = -1;
    sl::Transform IMAGE_TO_ARUCO_basis_change;
    IMAGE_TO_ARUCO_basis_change = sl::Transform::inverse(ARUCO_TO_IMAGE_basis_change);

    sl::POSITIONAL_TRACKING_STATE tracking_state;
    bool initialized = false;

    double x_acc, y_acc, z_acc, x_vel, y_vel, z_vel;

    auto start = std::chrono::high_resolution_clock::now();
    auto previousReset = std::chrono::high_resolution_clock::now();

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
    RCLCPP_INFO(nodeHandle->get_logger(), "Checking for crash");
    check_for_crash();

    RCLCPP_INFO(nodeHandle->get_logger(), "Starting camera");

    int writeCounter = 1;
    int resetCounter = 0;
	auto keySubscriber= nodeHandle->create_subscription<messages::msg::KeyState>("key",1,keyCallback);

    rclcpp::Rate rate(30);
    while (rclcpp::ok()) {
        if (zed.grab() == sl::ERROR_CODE::SUCCESS) {
            zed.retrieveImage(image_zed, sl::VIEW::LEFT, sl::MEM::CPU, image_size);
            cv::cvtColor(image_ocv, image_ocv_rgb, cv::COLOR_BGRA2BGR);
            cv::Mat grayImage;
            cv::cvtColor(image_ocv_rgb, grayImage, cv::COLOR_BGR2GRAY);
            cv::aruco::detectMarkers(image_ocv_rgb, dictionary, corners, ids);

            for (size_t i = 0; i < corners.size(); ++i) {
                cv::cornerSubPix(grayImage, corners[i], cv::Size(5, 5), cv::Size(-1, -1),
                                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
            }
            tracking_state = zed.getPosition(zedPose);
            std::string position_txt = "ZED  x: " + std::to_string(zedPose.pose_data.tx + 1.35) +
                     "; y: " + std::to_string(zedPose.pose_data.ty) +
                     "; z: " + std::to_string(zedPose.pose_data.tz);
            RCLCPP_INFO(nodeHandle->get_logger(), "%s", position_txt.c_str());
            // if at least one marker detected
            if (ids.size() > 0) {
                cv::aruco::estimatePoseSingleMarkers(corners, actual_marker_size_meters,
                                             camera_matrix, dist_coeffs, rvecs,
                                             tvecs);

                pose.setTranslation(sl::float3(tvecs[0](0), tvecs[0](1), tvecs[0](2)));
                pose.setRotationVector(
                    sl::float3(rvecs[0](0), rvecs[0](1), rvecs[0](2)));

                pose = IMAGE_TO_ARUCO_basis_change * pose;

                pose.inverse();
                auto user_coordinate_to_image = sl::getCoordinateTransformConversion4f(
                    init_params.coordinate_system, sl::COORDINATE_SYSTEM::IMAGE);
                can_reset = true;

                sl::Transform user_coordinate_to_ARUCO =
                    IMAGE_TO_ARUCO_basis_change * user_coordinate_to_image;
                sl::Transform ARUCO_to_user_coordinate =
                    sl::Transform::inverse(user_coordinate_to_ARUCO);

                pose = ARUCO_to_user_coordinate * pose * user_coordinate_to_ARUCO;
		        zedPosition.aruco_visible=true;
	        } 
            else {
	            zedPosition.aruco_visible=false;
                can_reset = false;
	        }

            if (zed.getSensorsData(sensors_data, sl::TIME_REFERENCE::IMAGE) == sl::ERROR_CODE::SUCCESS) {
                imu_data = sensors_data.imu;

                sl::float3 lin = imu_data.linear_acceleration;
                sl::float3 vel = imu_data.angular_velocity;
                x_acc = lin[0];
                y_acc = lin[1];
                z_acc = lin[2];
                x_vel = vel[0];
                y_vel = vel[1];
                z_vel = vel[2];
            }
            if (ids.size() == 0) {
                rvecs.clear();
                tvecs.clear();
                rvecs.resize(1);
                tvecs.resize(1);
            }
            
            if (!ids.empty() && !corners.empty() && can_reset && !has_reset && resetCounter < 2) {
                bool resetPose = isTagValidForReset(corners[0], cv::Size(image_zed.getWidth(), image_zed.getHeight()), auto_reset_aruco_screen_ratio);
                if (resetPose) {
                    RCLCPP_INFO(nodeHandle->get_logger(), "Reset pose");
                    zed.resetPositionalTracking(pose);
                    has_reset = true;
                    initialized = true;
                    resetCounter++;
                }
            }
            if (tracking_state == sl::POSITIONAL_TRACKING_STATE::OK) {
                zedPosition.x=zedPose.pose_data.tx + xOffset;
                zedPosition.y=zedPose.pose_data.ty;
                zedPosition.z=zedPose.pose_data.tz;
                zedPosition.ox=zedPose.getOrientation().ox;
                zedPosition.oy=zedPose.getOrientation().oy;
                zedPosition.oz=zedPose.getOrientation().oz;
                zedPosition.ow=zedPose.getOrientation().ow;
                zedPosition.roll = zedPose.pose_data.getEulerAngles(false).x - 12.33;
                zedPosition.pitch = zedPose.pose_data.getEulerAngles(false).y;
                zedPosition.yaw = zedPose.pose_data.getEulerAngles(false).z;
                zedPosition.x_acc = x_acc;
                zedPosition.y_acc = y_acc;
                zedPosition.z_acc = z_acc;
                zedPosition.x_vel = x_vel;
                zedPosition.y_vel = y_vel;
                zedPosition.z_vel = z_vel;
                zedPosition.aruco_initialized = initialized;
                zedPositionPublisher->publish(zedPosition);

                // Saves the position values to a file
                try{
                    if(writeCounter % 10 == 0){
                        RCLCPP_INFO(nodeHandle->get_logger(), "Before writing to file");
                        std::ostringstream oss;
                        oss << zedPose.pose_data.tx << "," << zedPose.pose_data.ty << "," << zedPose.pose_data.tz << ",";
                        oss << zedPose.pose_data.getEulerAngles(false).x << "," 
                            << zedPose.pose_data.getEulerAngles(false).y << "," 
                            << zedPose.pose_data.getEulerAngles(false).z;

                        std::string tmp_path = POSITION_FILE + ".tmp";
                        const std::string& data = oss.str();

                        int fd = open(tmp_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
                        if (fd == -1) {
                            RCLCPP_ERROR(nodeHandle->get_logger(), "Failed to open temp position file.");
                            continue;
                        }

                        ssize_t written = write(fd, data.c_str(), data.size());
                        if (written != (ssize_t)data.size()) {
                            RCLCPP_ERROR(nodeHandle->get_logger(), "Failed to write all data to temp file.");
                            close(fd);
                            continue;
                        }

                        if (fsync(fd) != 0) {
                            RCLCPP_ERROR(nodeHandle->get_logger(), "Failed to fsync temp file.");
                        }
                        close(fd);

                        // Rename after successful write + fsync
                        if (std::rename(tmp_path.c_str(), POSITION_FILE.c_str()) != 0) {
                            RCLCPP_ERROR(nodeHandle->get_logger(), "Failed to rename temp position file.");
                            std::perror("rename");
                        }
                    }
                }
                catch(std::exception e){
                    RCLCPP_INFO(nodeHandle->get_logger(), "Caught exception: %s", e.what());
                }
                
                writeCounter++;

            }

            if(!image_ocv_rgb.empty()){
                msg = cv_bridge::CvImage(hdr, "rgb8", image_ocv_rgb).toImageMsg();
                zedImagePublisher.publish(msg);
            }
    		auto finish = std::chrono::high_resolution_clock::now();
            if(std::chrono::duration_cast<std::chrono::milliseconds>(finish-previousReset).count() > 15000){
                has_reset = false;
                previousReset = std::chrono::high_resolution_clock::now();
            }

/*
            if(std::chrono::duration_cast<std::chrono::milliseconds>(finish-start).count() > 15000){
                RCLCPP_INFO(nodeHandle->get_logger(), "Before writing area map");
                // Save area map every 15 seconds
                // Might need to change this depending on the 
                zed.saveAreaMap(sl::String(AREA_MAP.c_str()));
                if (std::rename(TEMP_MAP.c_str(), AREA_MAP.c_str()) != 0) {
                    RCLCPP_ERROR(nodeHandle->get_logger(), "Failed to rename temp position file.");
                }
                start = std::chrono::high_resolution_clock::now();
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
