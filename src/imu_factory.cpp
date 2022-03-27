#include "ros2_imu_filter/imu_factory.h"

#include <functional>

namespace Rocket
{
    ImuFactory::ImuFactory(const rclcpp::NodeOptions &node_options)
        : Node("imu_filter", "/", node_options)
    {
        node = std::make_shared<ImuBaseNode>(*this);
    }

    ImuFactory::~ImuFactory()
    {
    }
}