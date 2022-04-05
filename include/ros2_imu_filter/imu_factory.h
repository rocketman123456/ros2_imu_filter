#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "ros2_imu_filter/imu_base_node.h"
#include "ros2_imu_filter/imu_complementary_node.h"
#include "ros2_imu_filter/imu_kalman_node.h"
#include "ros2_imu_filter/imu_mahony_node.h"

namespace Rocket
{
    using Result = rcl_interfaces::msg::SetParametersResult;

    class ImuFactory : public rclcpp::Node
    {
    public:
        ImuFactory(const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions());
        virtual ~ImuFactory();
    
    private:
        std::shared_ptr<ImuBaseNode> node_;
        std::string type_;
    };
}
