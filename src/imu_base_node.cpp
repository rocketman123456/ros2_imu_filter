#include "ros2_imu_filter/imu_base_node.h"

#include <functional>

namespace Rocket
{
    ImuBaseNode::ImuBaseNode(rclcpp::Node& node)
        : node(node)
    {
        if (node.has_parameter("accel_topic"))
        {
            node.get_parameter("accel_topic", accel_topic_);
        }
        else
        {
            accel_topic_ = node.declare_parameter("accel_topic", DEFAULT_ACCEL_TOPIC);
        }

        if (node.has_parameter("gyro_topic"))
        {
            node.get_parameter("gyro_topic", gyro_topic_);
        }
        else
        {
            gyro_topic_ = node.declare_parameter("gyro_topic", DEFAULT_GYRO_TOPIC);
        }

        if (node.has_parameter("accel_info_topic"))
        {
            node.get_parameter("accel_info_topic", accel_info_topic_);
        }
        else
        {
            accel_info_topic_ = node.declare_parameter("accel_info_topic", DEFAULT_ACCEL_INFO_TOPIC);
        }

        if (node.has_parameter("gyro_info_topic"))
        {
            node.get_parameter("gyro_info_topic", gyro_info_topic_);
        }
        else
        {
            gyro_info_topic_ = node.declare_parameter("gyro_info_topic", DEFAULT_GYRO_INFO_TOPIC);
        }

        std::function<void(const std_msgs::msg::String::SharedPtr)> simple_callback = 
            std::bind(&ImuBaseNode::topic_callback, this, std::placeholders::_1);
        subscription_ = node.create_subscription<std_msgs::msg::String>(
            "topic", 
            rclcpp::QoS(10), 
            simple_callback
        );

        std::function<void(const realsense_msgs::msg::IMUInfo::SharedPtr)> accel_info_callback = 
            std::bind(&ImuBaseNode::accel_info_topic_callback, this, std::placeholders::_1);
        accel_info_subscript_ = node.create_subscription<realsense_msgs::msg::IMUInfo>(
            accel_info_topic_, 
            rclcpp::QoS(10), 
            accel_info_callback
        );

        std::function<void(const realsense_msgs::msg::IMUInfo::SharedPtr)> gyro_info_callback = 
            std::bind(&ImuBaseNode::gyro_info_topic_callback, this, std::placeholders::_1);
        gyro_info_subscript_ = node.create_subscription<realsense_msgs::msg::IMUInfo>(
            gyro_info_topic_, 
            rclcpp::QoS(10), 
            gyro_info_callback
        );

        std::function<void(const sensor_msgs::msg::Imu::SharedPtr)> accel_callback = 
            std::bind(&ImuBaseNode::accel_topic_callback, this, std::placeholders::_1);
        accel_subscript_ = node.create_subscription<sensor_msgs::msg::Imu>(
            accel_topic_, 
            rclcpp::QoS(10), 
            accel_callback
        );

        std::function<void(const sensor_msgs::msg::Imu::SharedPtr)> gyro_callback = 
            std::bind(&ImuBaseNode::gyro_topic_callback, this, std::placeholders::_1);
        gyro_subscript_ = node.create_subscription<sensor_msgs::msg::Imu>(
            gyro_topic_, 
            rclcpp::QoS(10), 
            gyro_callback
        );
    }

    ImuBaseNode::~ImuBaseNode()
    {
    }

    void ImuBaseNode::topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(node.get_logger(), "I heard: '%s'", msg->data.c_str());
    }

    void ImuBaseNode::accel_info_topic_callback(const realsense_msgs::msg::IMUInfo::SharedPtr msg) const
    {
        static int64_t count = 0;
        if(count % 1000 == 0)
            RCLCPP_INFO(node.get_logger(), "I heard: 'accel info topic %d'", count);
        count++;
    }

    void ImuBaseNode::gyro_info_topic_callback(const realsense_msgs::msg::IMUInfo::SharedPtr msg) const
    {
        static int64_t count = 0;
        if(count % 1000 == 0)
            RCLCPP_INFO(node.get_logger(), "I heard: 'gyro info topic %d'", count);
        count++;
    }

    void ImuBaseNode::accel_topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg) const
    {
        static int64_t count = 0;
        if(count % 1000 == 0)
            RCLCPP_INFO(node.get_logger(), "I heard: 'accel topic %d'", count);
        count++;
    }

    void ImuBaseNode::gyro_topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg) const
    {
        static int64_t count = 0;
        if(count % 1000 == 0)
            RCLCPP_INFO(node.get_logger(), "I heard: 'gyro topic %d'", count);
        count++;
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
                //TODO
            }
            else if (param_name == "gyro_topic")
            {
                //TODO
            }
            else if (param_name == "accel_info_topic")
            {
                //TODO
            }
            else if (param_name == "gyro_info_topic")
            {
                //TODO
            }
        }
        return result;
    }
}