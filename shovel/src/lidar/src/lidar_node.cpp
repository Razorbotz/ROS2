#include <rclcpp/rclcpp.hpp>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <cstring>
#include "messages/msg/lidar_distance.hpp"

int init_can_socket(const char* ifname, canid_t target_id) {
    int socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd < 0) return -1;

    struct ifreq ifr;
    std::strcpy(ifr.ifr_name, ifname);
    ioctl(socket_fd, SIOCGIFINDEX, &ifr);

    struct can_filter rfilter[1];
    rfilter[0].can_id   = target_id;
    rfilter[0].can_mask = CAN_SFF_MASK;
    setsockopt(socket_fd, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000; 
    setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(socket_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(socket_fd);
        return -1;
    }

    return socket_fd;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("lidar");
    auto distance_pub = node->create_publisher<messages::msg::LidarDistance>("lidar_distance", 10);
    int socket_fd = init_can_socket("can0", 2);
    if (socket_fd < 0) {
        RCLCPP_ERROR(node->get_logger(), "Failed to bind to can0 or apply filter.");
        rclcpp::shutdown();
        return -1;
    }

    RCLCPP_INFO(node->get_logger(), "Listening on can0. Kernel filter applied for ID 2.");
    struct can_frame frame;

    while (rclcpp::ok()) {
        
        int nbytes = read(socket_fd, &frame, sizeof(struct can_frame));

        if (nbytes > 0 && frame.can_dlc >= 2) {
            
            uint16_t distance_mm = frame.data[0] | (frame.data[1] << 8);
            float distance_m = static_cast<float>(distance_mm) / 1000.0f;

            auto distance_msg = messages::msg::LidarDistance();
            distance_msg.distance_mm = distance_mm;
            distance_msg.distance_m = distance_m;

            distance_pub->publish(distance_msg);
        }
        rclcpp::spin_some(node);
    }

    close(socket_fd);
    rclcpp::shutdown();
    return 0;
}