#pragma once

#include <memory>
#include <string>
#include <mutex>
#include <vector>

#include <Eigen/Eigen>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "ros2_imu_filter/imu_constant.h"

namespace Rocket
{
    using Result = rcl_interfaces::msg::SetParametersResult;

    class ImuBaseNode
    {
    public:
        ImuBaseNode(rclcpp::Node& node);
        virtual ~ImuBaseNode() = default;

    protected:
        virtual void Filter() {}

    private:
        void UpdateAccelData(const sensor_msgs::msg::Imu::SharedPtr msg);
        void UpdateGyroData(const sensor_msgs::msg::Imu::SharedPtr msg);

        void HandleAccelData(const sensor_msgs::msg::Imu::SharedPtr msg);
        void HandleGyroData(const sensor_msgs::msg::Imu::SharedPtr msg);

    private:
        void topic_callback(const std_msgs::msg::String::SharedPtr msg) const;
        void accel_topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
        void gyro_topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

        Result param_change_callback(const std::vector<rclcpp::Parameter> &params);

    protected:
        rclcpp::Node& node_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr accel_subscript_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr gyro_subscript_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
        std::string accel_topic_;
        std::string gyro_topic_;
        int64_t accel_msg_count = 0;
        int64_t gyro_msg_count = 0;
        bool update_on_accel_ = true;
        bool update_on_gyro_ = true;

    protected:
        Eigen::Matrix3d rotation_ = Eigen::Matrix3d::Identity();
        Eigen::Vector3d accel_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d gyro_ = Eigen::Vector3d::Zero();
        double last_time_ms_;
        double dt_ms_ = 0;

        std::mutex mutex_;
    };
}
