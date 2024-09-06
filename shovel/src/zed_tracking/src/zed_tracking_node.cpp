
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

#define ROW_COUNT 10
rclcpp::Node::SharedPtr nodeHandle;
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

/** @brief String parameter function
 * 
 * Function that takes a string as a parameter containing the
 * name of the parameter that is being parsed from the launch
 * file and the initial value of the parameter as inputs, then
 * gets the parameter, casts it as a string, displays the value
 * of the parameter on the command line and the log file, then
 * returns the parsed value of the parameter.
 * @param parametername String of the name of the parameter
 * @param initialValue Initial value of the parameter
 * @return value Value of the parameter
 * */
template <typename T>
T getParameter(std::string parameterName, std::string initialValue){
	nodeHandle->declare_parameter<T>(parameterName, initialValue);
	rclcpp::Parameter param = nodeHandle->get_parameter(parameterName);
	T value = param.as_string();
	std::cout << parameterName << ": " << value << std::endl;
	std::string output = parameterName + ": " + value;
	RCLCPP_INFO(nodeHandle->get_logger(), output.c_str());
	return value;
}


int main(int argc, char **argv) {
    rclcpp::init(argc,argv);
    nodeHandle = rclcpp::Node::make_shared("zed_tracking");

    RCLCPP_INFO(nodeHandle->get_logger(),"Starting zed_tracking");

    std::string resolution = getParameter<std::string>("resolution", "VGA");
    messages::msg::ZedPosition zedPosition;
    auto zedPositionPublisher=nodeHandle->create_publisher<messages::msg::ZedPosition>("zed_position",1);

    image_transport::ImageTransport it(nodeHandle);
    image_transport::Publisher zedImagePublisher = it.advertise("zed_image", 1);
    std_msgs::msg::Header hdr;
    sensor_msgs::msg::Image::SharedPtr msg;

    // Create a ZED camera object
    sl::Camera zed;

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
        init_params.camera_resolution = sl::RESOLUTION::VGA;
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
        init_params.camera_fps = 30; 
    }
    else{
        init_params.camera_resolution = sl::RESOLUTION::VGA;
        init_params.camera_fps = 30; 
    }
    init_params.coordinate_units = sl::UNIT::METER;
    init_params.coordinate_system = sl::COORDINATE_SYSTEM::IMAGE;
//    init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
//    init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP;
//    init_params.coordinate_system = sl::COORDINATE_SYSTEM::LEFT_HANDED_Y_UP;
//    init_params.coordinate_system = sl::COORDINATE_SYSTEM::LEFT_HANDED_Z_UP;
//    init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
    init_params.sensors_required = true;

    // Open the camera
    sl::ERROR_CODE err = zed.open(init_params);
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

    sl::Transform arucoPose;
    sl::Pose zedPose;
    std::vector<cv::Vec3d> rvecs, tvecs;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    std::string zed_position_txt;
    std::string zed_rotation_txt;
    std::string aruco_position_txt;
    sl::float3 angles;

    sl::SensorsData sensors_data;
    sl::SensorsData::IMUData imu_data;

    sl::Transform ARUCO_TO_IMAGE_basis_change;
    ARUCO_TO_IMAGE_basis_change.r00 = -1;
    ARUCO_TO_IMAGE_basis_change.r11 = -1;
    sl::Transform IMAGE_TO_ARUCO_basis_change;
    IMAGE_TO_ARUCO_basis_change = sl::Transform::inverse(ARUCO_TO_IMAGE_basis_change);

    bool initialized = false;

    double x_acc, y_acc, z_acc, x_vel, y_vel, z_vel;

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    sl::Mat depth_image;
    sl::Mat depth_map;

    int currentRow=0;
    float pastValues[ROW_COUNT][7];
    float average[7];
    for(int col=0;col<7;col++){
	    average[col]=0;
        for(int row=0;row<ROW_COUNT;row++){
	        pastValues[row][col]=0;
	    }
    }

    sl::PositionalTrackingParameters tracking_params;
    tracking_params.enable_imu_fusion = true;
    tracking_params.enable_area_memory = true;
    auto returned_state = zed.enablePositionalTracking(tracking_params);
    if (returned_state != sl::ERROR_CODE::SUCCESS) {
        zed.close();
        return EXIT_FAILURE;
    }

    rclcpp::Rate rate(30);
    while (rclcpp::ok()) {
        if (zed.grab() == sl::ERROR_CODE::SUCCESS) {
            // Retrieve the left image
            zed.retrieveImage(image_zed, sl::VIEW::LEFT, sl::MEM::CPU, image_size);
            zed.retrieveImage(depth_image, sl::VIEW::DEPTH);
            zed.retrieveMeasure(depth_map, sl::MEASURE::DEPTH);

            cv::Mat fullImage = cv::Mat((int) depth_image.getHeight(), (int) depth_image.getWidth(), CV_8UC4, depth_image.getPtr<sl::uchar1>(sl::MEM::CPU));
            // Remove part of image that holds the robot arms
            cv::Rect roi(300, 30, depth_image.getWidth() - 600, depth_image.getHeight() - 80);

            cv::Mat cvImage = fullImage(roi);
            cv::Mat canny;
            cv::Canny(cvImage, canny, 5, 30, 3);
            cv::findContours(canny, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

            // convert to RGB
            cv::cvtColor(image_ocv, image_ocv_rgb, cv::COLOR_RGBA2RGB);
            
            cv::Mat grayImage;
            cv::cvtColor(image_ocv_rgb, grayImage, cv::COLOR_BGR2GRAY);

            // detect marker
            cv::aruco::detectMarkers(image_ocv_rgb, dictionary, corners, ids);

            for(size_t i = 0; i < corners.size(); ++i){
                cv::cornerSubPix(grayImage, corners[i], cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
            }

            sl::POSITIONAL_TRACKING_STATE tracking_state = zed.getPosition(zedPose);
            
            // if at least one marker detected
            if (ids.size() > 0) {
	        //	int id=ids[0];
                cv::aruco::estimatePoseSingleMarkers(corners, actual_marker_size_meters, camera_matrix, dist_coeffs, rvecs, tvecs);
                arucoPose.setTranslation(sl::float3(tvecs[0](0), tvecs[0](1), tvecs[0](2)));
                arucoPose.setRotationVector(sl::float3(rvecs[0](0), rvecs[0](1), rvecs[0](2)));
                arucoPose = IMAGE_TO_ARUCO_basis_change * arucoPose;
                arucoPose.inverse();
                auto user_coordinate_to_image = sl::getCoordinateTransformConversion4f(init_params.coordinate_system, sl::COORDINATE_SYSTEM::IMAGE);
                sl::Transform user_coordinate_to_aruco = IMAGE_TO_ARUCO_basis_change * user_coordinate_to_image;
                sl::Transform aruco_to_user_coordinate = sl::Transform::inverse(user_coordinate_to_aruco);

                arucoPose = aruco_to_user_coordinate * arucoPose * user_coordinate_to_aruco;

                zedPosition.aruco_roll = arucoPose.getEulerAngles(false).x;
                zedPosition.aruco_pitch = arucoPose.getEulerAngles(false).y;
                zedPosition.aruco_yaw = arucoPose.getEulerAngles(false).z;
		        zedPosition.aruco_visible=true;
                if(!initialized){
                    zed.resetPositionalTracking(arucoPose);
                    initialized = true;                
                }
	        } 
            else {
	            zedPosition.aruco_visible=false;
	        }

            zed.getSensorsData(sensors_data, sl::TIME_REFERENCE::IMAGE);

            imu_data = sensors_data.imu;

            sl::float3 lin = imu_data.linear_acceleration;
            sl::float3 vel = imu_data.angular_velocity;
            x_acc = lin[0];
            y_acc = lin[1];
            z_acc = lin[2];
            x_vel = vel[0];
            y_vel = vel[1];
            z_vel = vel[2];

            for(size_t i = 0; i < contours.size(); i++){
                if(cv::contourArea(contours[i]) > 20){
                    cv::Rect box = boundingRect(contours[i]);
                    float depth_value = 0;
                    depth_map.getValue(int(box.x + box.width / 2)+300, int(box.y + box.height / 2) + 50, &depth_value);
                    float calcWidth = box.width * depth_value * 0.00018474;
                    if(calcWidth > 20){
                        float x = depth_value * sin(zedPose.getEulerAngles(false).y) / sin(90.0) + calcWidth;
                        float y = depth_value * sin(90- zedPose.getEulerAngles(false).y) / sin(90.0) + calcWidth;
                        // publish values here
                    }
                }
            }

/*
            zed.retrieveImage(image_zed, sl::VIEW::LEFT);
            zed.retrieveMeasure(depth, sl::MEASURE::DEPTH);
            zed.retrieveMeasure(point_cloud, sl::MEASURE::XYZRGBA);

            int x = image_zed.getWidth() / 2;
            int y = image_zed.getHeight() / 2;
            sl::float4 point_cloud_value;
            point_cloud.getValue(x, y, &point_cloud_value);

            if(std::isfinite(point_cloud_value.z)){
                distance = sqrt(point_cloud_value.x * point_cloud_value.x + point_cloud_value.y * point_cloud_value.y + point_cloud_value.z * point_cloud_value.z);
                distance *= 10;
                RCLCPP_INFO(nodeHandle->get_logger(), "Distance: %f", distance);
            }
            else{
                distance = -1;
            }
*/

        if (tracking_state == sl::POSITIONAL_TRACKING_STATE::OK) {
            zedPosition.x=zedPose.getTranslation().x;
            zedPosition.y=zedPose.getTranslation().y;
            zedPosition.z=zedPose.getTranslation().z;
            zedPosition.ox=zedPose.getOrientation().ox;
            zedPosition.oy=zedPose.getOrientation().oy;
            zedPosition.oz=zedPose.getOrientation().oz;
            zedPosition.ow=zedPose.getOrientation().ow;
            zedPosition.roll = zedPose.getEulerAngles(false).x;
            zedPosition.pitch = zedPose.getEulerAngles(false).y;
            zedPosition.yaw = zedPose.getEulerAngles(false).z;
            zedPosition.x_acc = x_acc;
            zedPosition.y_acc = y_acc;
            zedPosition.z_acc = z_acc;
            zedPosition.x_vel = x_vel;
            zedPosition.y_vel = y_vel;
            zedPosition.z_vel = z_vel;
            zedPosition.aruco_initialized = initialized;
            zedPositionPublisher->publish(zedPosition);
        }

	    if(!image_ocv_rgb.empty()){
	        msg = cv_bridge::CvImage(hdr, "rgb8", image_ocv_rgb).toImageMsg();
            zedImagePublisher.publish(msg);
	    }

        }
	    rate.sleep();
    }
    zed.close();
    return 0;

}
