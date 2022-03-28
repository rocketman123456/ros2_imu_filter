#pragma once

#include <memory>
#include <string>
#include <mutex>
#include <vector>

#include <Eigen/Eigen>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"

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
        void UpdateAccelData(const sensor_msgs::msg::Imu::SharedPtr msg);
        void UpdateGyroData(const sensor_msgs::msg::Imu::SharedPtr msg);

        void HandleAccelData(const sensor_msgs::msg::Imu::SharedPtr msg);
        void HandleGyroData(const sensor_msgs::msg::Imu::SharedPtr msg);

    private:
        void topic_callback(const std_msgs::msg::String::SharedPtr msg) const;
        void accel_topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
        void gyro_topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

        Result param_change_callback(const std::vector<rclcpp::Parameter> &params);

    private:
        rclcpp::Node& node_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr accel_subscript_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr gyro_subscript_;
        std::string accel_topic_;
        std::string gyro_topic_;
        bool update_on_accel_ = true;
        bool update_on_gyro_ = true;

    protected:
        // TODO define data type
        Eigen::Vector3d accel;
        Eigen::Vector3d gyro;
        double last_time_ms;

        std::mutex mutex_;
    };
}
