
#pragma once

#include <memory>

#include "AHRS.h"
#include <rev/RelativeEncoder.h>

typedef struct sensors_t {
    // navX IMU
    std::unique_ptr<AHRS> ahrs;

} SensorInterface;
