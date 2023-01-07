
#pragma once

#include <memory>

#include "AHRS.h"
#include <frc/DigitalInput.h>
#include <frc/AnalogInput.h>
#include <rev/RelativeEncoder.h>

// == sensor ports ==
#define DEPLOY_LIMIT_SWITCH 2
#define RETRACTED_LIMIT_SWITCH 3

#define BALL_DETECTOR 9

#define LEFT_FRONT_STEER_ENCODER 0
#define LEFT_BACK_STEER_ENCODER 1
#define RIGHT_FRONT_STEER_ENCODER 2
#define RIGHT_BACK_STEER_ENCODER 3

typedef struct sensors_t {
    // navX IMU
    std::unique_ptr<AHRS> ahrs;

    // intake limit switches
    std::unique_ptr<frc::DigitalInput> deploy_limit_switch;
    std::unique_ptr<frc::DigitalInput> retracted_limit_switch;

    // swerve drive steer encoders
    std::unique_ptr<frc::AnalogInput> left_front_steer_encoder;
    std::unique_ptr<frc::AnalogInput> left_back_steer_encoder;
    std::unique_ptr<frc::AnalogInput> right_front_steer_encoder;
    std::unique_ptr<frc::AnalogInput> right_back_steer_encoder;

    // Shooter ball detector
    std::unique_ptr<frc::DigitalInput> ball_detector;
} SensorInterface;
