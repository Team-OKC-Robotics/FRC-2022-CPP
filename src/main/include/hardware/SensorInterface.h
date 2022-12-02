
#pragma once

#include <memory>

#include "AHRS.h"
#include <rev/RelativeEncoder.h>
#include <frc/DigitalInput.h>

// == sensor ports ==
#define DEPLOY_LIMIT_SWITCH 2
#define RETRACTED_LIMIT_SWITCH 3

typedef struct sensors_t {
    // navX IMU
    std::unique_ptr<AHRS> ahrs;

    // intake limit switches
    std::unique_ptr<frc::DigitalInput> deploy_limit_switch;
    std::unique_ptr<frc::DigitalInput> retracted_limit_switch;
} SensorInterface;
