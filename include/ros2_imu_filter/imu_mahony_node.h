#pragma once

#include "rclcpp/rclcpp.hpp"

#include "ros2_imu_filter/imu_base_node.h"

namespace Rocket {
    class ImuMahonyNode : public ImuBaseNode {
    public:
        ImuMahonyNode(rclcpp::Node& node) : ImuBaseNode(node) {}
        virtual ~ImuMahonyNode() = default;

    protected:
        void Filter() override;
    };
}
