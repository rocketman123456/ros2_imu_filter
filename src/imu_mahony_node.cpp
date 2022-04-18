#include "ros2_imu_filter/imu_mahony_node.h"

namespace Rocket {
    void ImuMahonyNode::Filter() {
        // First wait for stable data
        if(accel_msg_count < WAIT_COUNT || gyro_msg_count < WAIT_COUNT) {
            return;
        }
        
    }
}
