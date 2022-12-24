
#pragma once

#include <memory>

#include "AHRS.h"
#include <rev/RelativeEncoder.h>
#include <frc/DigitalInput.h>
#include <frc/AnalogInput.h>

// == sensor ports ==
#define DEPLOY_LIMIT_SWITCH 2
#define RETRACTED_LIMIT_SWITCH 3

typedef struct sensors_t {
    // navX IMU
    std::unique_ptr<AHRS> ahrs;

    // intake limit switches
    std::unique_ptr<frc::DigitalInput> deploy_limit_switch;
    std::unique_ptr<frc::DigitalInput> retracted_limit_switch;

    std::unique_ptr<frc::AnalogInput> left_front_steer_encoder;
    std::unique_ptr<frc::AnalogInput> left_back_steer_encoder;
    std::unique_ptr<frc::AnalogInput> right_front_steer_encoder;
    std::unique_ptr<frc::AnalogInput> right_back_steer_encoder;
} SensorInterface;
