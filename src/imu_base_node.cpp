#include "ros2_imu_filter/imu_base_node.h"

#include <functional>

namespace Rocket
{
    

    ImuBaseNode::ImuBaseNode(rclcpp::Node& node)
        : node_(node)
    {
        // accel_topic
        if (node_.has_parameter("accel_topic"))
            node_.get_parameter("accel_topic", accel_topic_);
        else
            accel_topic_ = node_.declare_parameter("accel_topic", DEFAULT_ACCEL_TOPIC);
        // gyro_topic
        if (node_.has_parameter("gyro_topic"))
            node_.get_parameter("gyro_topic", gyro_topic_);
        else
            gyro_topic_ = node_.declare_parameter("gyro_topic", DEFAULT_GYRO_TOPIC);
        // update_on_accel
        if (node_.has_parameter("update_on_accel"))
            node_.get_parameter("update_on_accel", update_on_accel_);
        else
            update_on_accel_ = node_.declare_parameter("update_on_accel", false);
        // update_on_gyro
        if (node_.has_parameter("update_on_gyro"))
            node_.get_parameter("update_on_gyro", update_on_gyro_);
        else
            update_on_gyro_ = node_.declare_parameter("update_on_gyro", true);

        // Register Callback
        std::function<void(const std_msgs::msg::String::SharedPtr)> simple_callback = 
            std::bind(&ImuBaseNode::topic_callback, this, std::placeholders::_1);
        subscription_ = node_.create_subscription<std_msgs::msg::String>(
            "topic", 
            rclcpp::QoS(10), 
            simple_callback
        );

        std::function<void(const sensor_msgs::msg::Imu::SharedPtr)> accel_callback = 
            std::bind(&ImuBaseNode::accel_topic_callback, this, std::placeholders::_1);
        accel_subscript_ = node_.create_subscription<sensor_msgs::msg::Imu>(
            accel_topic_, 
            rclcpp::QoS(10), 
            accel_callback
        );

        std::function<void(const sensor_msgs::msg::Imu::SharedPtr)> gyro_callback = 
            std::bind(&ImuBaseNode::gyro_topic_callback, this, std::placeholders::_1);
        gyro_subscript_ = node_.create_subscription<sensor_msgs::msg::Imu>(
            gyro_topic_, 
            rclcpp::QoS(10), 
            gyro_callback
        );

        callback_handle_ = node_.add_on_set_parameters_callback(
            std::bind(&ImuBaseNode::param_change_callback, this, std::placeholders::_1)
        );

        // auto header = std_msgs::msg::Header();
        // last_time_ms_ = header.stamp.sec * 1000.0 + header.stamp.nanosec * 0.001 * 0.001;
    }

    void ImuBaseNode::topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(node_.get_logger(), "I heard: '%s'", msg->data.c_str());
    }

    void ImuBaseNode::accel_topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        UpdateAccelData(msg);
        if(update_on_accel_) {
            HandleAccelData(msg);
            Filter();
        }
        ++accel_msg_count;
    }

    void ImuBaseNode::gyro_topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        UpdateGyroData(msg);
        if(update_on_gyro_) {
            HandleGyroData(msg);
            Filter();
        }
        ++gyro_msg_count;
    }

    void ImuBaseNode::UpdateAccelData(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        accel_ << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;

        accel_covariance_ << 
            msg->linear_acceleration_covariance[0], 
            msg->linear_acceleration_covariance[1], 
            msg->linear_acceleration_covariance[2], 
            msg->linear_acceleration_covariance[3], 
            msg->linear_acceleration_covariance[4], 
            msg->linear_acceleration_covariance[5], 
            msg->linear_acceleration_covariance[6], 
            msg->linear_acceleration_covariance[7], 
            msg->linear_acceleration_covariance[8];
    }

    void ImuBaseNode::HandleAccelData(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        double current_time_ms = msg->header.stamp.sec * 1000.0 + msg->header.stamp.nanosec * 0.001 * 0.001;
        dt_ms_ = current_time_ms - last_time_ms_;
        last_time_ms_ = current_time_ms;
    }

    void ImuBaseNode::UpdateGyroData(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        gyro_ << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;

        gyro_covariance_ << 
            msg->angular_velocity_covariance[0], 
            msg->angular_velocity_covariance[1], 
            msg->angular_velocity_covariance[2], 
            msg->angular_velocity_covariance[3], 
            msg->angular_velocity_covariance[4], 
            msg->angular_velocity_covariance[5], 
            msg->angular_velocity_covariance[6], 
            msg->angular_velocity_covariance[7], 
            msg->angular_velocity_covariance[8];
    }

    void ImuBaseNode::HandleGyroData(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        double current_time_ms = msg->header.stamp.sec * 1000.0 + msg->header.stamp.nanosec * 0.001 * 0.001;
        dt_ms_ = current_time_ms - last_time_ms_;
        last_time_ms_ = current_time_ms;
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
            else if (param_name == "update_on_accel")
            {
                update_on_accel_ = param.as_bool();
            }
            else if (param_name == "update_on_gyro")
            {
                update_on_gyro_ = param.as_bool();
            }
        }
        return result;
    }
}