#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "realsense_msgs/msg/imu_info.hpp"

#include "ros2_imu_filter/imu_constant.h"

namespace Rocket
{
    using Result = rcl_interfaces::msg::SetParametersResult;

    class ImuBaseNode : public rclcpp::Node
    {
    public:
        ImuBaseNode();
        virtual ~ImuBaseNode();

    private:
        void topic_callback(const std_msgs::msg::String::SharedPtr msg) const;
        void accel_topic_callback(const realsense_msgs::msg::IMUInfo::SharedPtr msg) const;
        void gyro_topic_callback(const realsense_msgs::msg::IMUInfo::SharedPtr msg) const;

        Result param_change_callback(const std::vector<rclcpp::Parameter> &params);

    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
        rclcpp::Subscription<realsense_msgs::msg::IMUInfo>::SharedPtr accel_subscript_;
        rclcpp::Subscription<realsense_msgs::msg::IMUInfo>::SharedPtr gyro_subscript_;
        std::string accel_topic_;
        std::string gyro_topic_;
    };
}
