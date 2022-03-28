#pragma once

#include "rclcpp/rclcpp.hpp"

#include "ros2_imu_filter/imu_base_node.h"

namespace Rocket {
    class ImuComplementaryNode : public ImuBaseNode {
    public:
        ImuComplementaryNode(rclcpp::Node& node) : ImuBaseNode(node) {}
        virtual ~ImuComplementaryNode() = default;

    protected:
        void Filter() override;
    };
}
