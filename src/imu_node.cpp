#include <memory>

#include "ros2_imu_filter/imu_base_node.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto imu_node = std::make_shared<Rocket::ImuBaseNode>();
    rclcpp::spin(imu_node);
    rclcpp::shutdown();
}