#include <memory>

#include "ros2_imu_filter/imu_factory.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto imu_node = std::make_shared<Rocket::ImuFactory>();
    rclcpp::spin(imu_node);
    rclcpp::shutdown();
    return 0;
}
