#pragma once

#include "rclcpp/rclcpp.hpp"

#include "ros2_imu_filter/imu_base_node.h"

namespace Rocket {
    class ImuKalmanNode : public ImuBaseNode {
    public:
        ImuKalmanNode(rclcpp::Node& node) : ImuBaseNode(node) {}
        virtual ~ImuKalmanNode() = default;

    protected:
        void Filter() override;
    };
}
