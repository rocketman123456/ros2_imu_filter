#include "ros2_imu_filter/imu_base_node.h"

#include <functional>

namespace Rocket
{
    ImuBaseNode::ImuBaseNode()
        : Node("minimal_subscriber")
    {
        if (this->has_parameter("accel_topic"))
        {
            this->get_parameter("accel_topic", accel_topic_);
        }
        else
        {
            accel_topic_ = this->declare_parameter("accel_topic", DEFAULT_ACCEL_TOPIC);
        }

        if (this->has_parameter("gyro_topic"))
        {
            this->get_parameter("gyro_topic", gyro_topic_);
        }
        else
        {
            gyro_topic_ = this->declare_parameter("gyro_topic", DEFAULT_GYRO_TOPIC);
        }

        std::function<void(const std_msgs::msg::String::SharedPtr)> callback_fn = 
            std::bind(&ImuBaseNode::topic_callback, this, std::placeholders::_1);
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "topic", 
            rclcpp::QoS(10), 
            callback_fn
        );
    }

    ImuBaseNode::~ImuBaseNode()
    {
    }

    void ImuBaseNode::topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }

    void ImuBaseNode::accel_topic_callback(const realsense_msgs::msg::IMUInfo::SharedPtr msg) const
    {
    }

    void ImuBaseNode::gyro_topic_callback(const realsense_msgs::msg::IMUInfo::SharedPtr msg) const
    {
    }

    Result ImuBaseNode::param_change_callback(const std::vector<rclcpp::Parameter> &params)
    {
        auto result = Result();
        result.successful = true;
        for (auto &param : params)
        {
            auto param_name = param.get_name();
            if (param_name == "accel_topic")
            {
                //result = toggleStream(COLOR, param);
            }
            else if (param_name == "gyro_topic")
            {
                //result = changeResolution(COLOR, param);
            }
            
        }
        return result;
    }
}