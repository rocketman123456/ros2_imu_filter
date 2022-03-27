#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "ros2_imu_filter/imu_base_node.h"

namespace Rocket
{
    using Result = rcl_interfaces::msg::SetParametersResult;

    class ImuFactory : public rclcpp::Node
    {
    public:
        ImuFactory(const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions());
        virtual ~ImuFactory();
    
    private:
        std::shared_ptr<ImuBaseNode> node;
    };
}
