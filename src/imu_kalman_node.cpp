#include "ros2_imu_filter/imu_kalman_node.h"
#include "ros2_imu_filter/kalman_filter.h"

namespace Rocket {
    void ImuKalmanNode::Filter() {
        // First wait for stable data
        if(accel_msg_count < WAIT_COUNT || gyro_msg_count < WAIT_COUNT) {
            return;
        }
        if(accel_msg_count % 1000 == 0) {
            RCLCPP_INFO(node_.get_logger(), "dt : %2.4f, accel : %2.4f, %2.4f, %2.4f, gyro : %2.4f, %2.4f, %2.4f", 
                dt_ms_, accel_[0], accel_[1], accel_[2], gyro_[0], gyro_[1], gyro_[2]);
        }
        
    }
}
