
#pragma once

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <networktables/NetworkTableEntry.h>

#include "Parameters.h"

namespace DrivetrainUI {
    // Get the tab
    extern frc::ShuffleboardTab &nt_tab;

    // Add all the defaults
    // Write mode
    extern nt::GenericEntry *const nt_write_mode;

    // Encoder
    extern nt::GenericEntry *const nt_left_ticks;
    extern nt::GenericEntry *const nt_right_ticks;
    extern nt::GenericEntry *const nt_total_ticks;
    extern nt::GenericEntry *const nt_dist_error;

    // Distance PID
    extern nt::GenericEntry *const nt_dist_kp;
    extern nt::GenericEntry *const nt_dist_ki;
    extern nt::GenericEntry *const nt_dist_kd;

    // Heading PID
    extern nt::GenericEntry *const nt_heading;
    extern nt::GenericEntry *const nt_heading_kp;
    extern nt::GenericEntry *const nt_heading_ki;
    extern nt::GenericEntry *const nt_heading_kd;

    // Turn PID
    extern nt::GenericEntry *const nt_turn_kp;
    extern nt::GenericEntry *const nt_turn_ki;
    extern nt::GenericEntry *const nt_turn_kd;

    // Gyro
    extern nt::GenericEntry *const nt_reset_gyro;

    // Save drivetrain parameters
    extern nt::GenericEntry *const nt_save;
} // namespace DrivetrainUI

namespace ShooterUI {
    extern frc::ShuffleboardTab &nt_tab;

    // Write mode
    extern nt::GenericEntry *const nt_write_mode;

    // sensors
    extern nt::GenericEntry *const nt_ticks;
    extern nt::GenericEntry *const nt_rpm;
    extern nt::GenericEntry *const nt_output;
    extern nt::GenericEntry *const nt_setpoint;
    extern nt::GenericEntry *const nt_vel_err;
    extern nt::GenericEntry *const nt_has_ball;

    // PID
    extern nt::GenericEntry *const nt_shoot_kp;
    extern nt::GenericEntry *const nt_shoot_ki;
    extern nt::GenericEntry *const nt_shoot_kd;

    // Status
    extern nt::GenericEntry *const nt_shooter_good;

    // Presets
    extern nt::GenericEntry *const nt_normal_shot;
    extern nt::GenericEntry *const nt_against_hub;
    extern nt::GenericEntry *const nt_low_goal;
    extern nt::GenericEntry *const nt_far_shot;
} // namespace ShooterUI
