
#pragma once

#include <memory>
#include <rev/CANSparkMax.h>

// Brushless Motor
#define BRUSHLESS rev::CANSparkMax::MotorType::kBrushless

// Coast
#define COAST rev::CANSparkMax::IdleMode::kCoast

// Motor IDs
#define LEFT_MOTOR_1 1
#define LEFT_MOTOR_2 2
#define LEFT_MOTOR_3 3
#define RIGHT_MOTOR_1 4
#define RIGHT_MOTOR_2 5
#define RIGHT_MOTOR_3 6

typedef struct actuator_interface_t
{
    // Left drivetrain motors
    std::unique_ptr<rev::CANSparkMax> left_motor_1;
    std::unique_ptr<rev::CANSparkMax> left_motor_2;
    std::unique_ptr<rev::CANSparkMax> left_motor_3;

    // Right drivetrain motors
    std::unique_ptr<rev::CANSparkMax> right_motor_1;
    std::unique_ptr<rev::CANSparkMax> right_motor_2;
    std::unique_ptr<rev::CANSparkMax> right_motor_3;
} ActuatorInterface;