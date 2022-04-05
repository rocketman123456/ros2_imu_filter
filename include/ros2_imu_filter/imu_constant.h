#pragma once

#include <string>

namespace Rocket
{
    const std::string DEFAULT_ACCEL_TOPIC = "/d435i/camera/accel/sample";
    const std::string DEFAULT_GYRO_TOPIC = "/d435i/camera/gyro/sample";
    const std::string DEFAULT_TYPE = "complementary";
    const int64_t WAIT_COUNT = 50;
}
