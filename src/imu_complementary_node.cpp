#include "ros2_imu_filter/imu_complementary_node.h"

namespace Rocket {
    void ImuComplementaryNode::Filter() {
        // First wait for stable data
        if(accel_msg_count < 5 || gyro_msg_count < 5) {
            return;
        }
    }
}
