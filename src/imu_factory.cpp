#include "ros2_imu_filter/imu_factory.h"

#include <functional>

namespace Rocket
{
    ImuFactory::ImuFactory(const rclcpp::NodeOptions &node_options)
        : Node("imu_filter", "/", node_options)
    {
        if (this->has_parameter("type"))
            this->get_parameter("type", type_);
        else
            type_ = this->declare_parameter("type", DEFAULT_TYPE);

        if(type_ == "empty") {
            node_ = std::make_shared<ImuBaseNode>(*this);
        } else if(type_ == "kalman") {
            
        } else if(type_ == "complementary") {
            node_ = std::make_shared<ImuComplementaryNode>(*this);
        } else if(type_ == "mahony") {

        }
    }

    ImuFactory::~ImuFactory()
    {
    }
}