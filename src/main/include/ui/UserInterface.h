
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
    extern nt::GenericEntry &nt_write_mode;

    // Encoder
    extern nt::GenericEntry &nt_left_ticks;
    extern nt::GenericEntry &nt_right_ticks;
    extern nt::GenericEntry &nt_total_ticks;
    extern nt::GenericEntry &nt_dist_error;

    // Distance PID
    extern nt::GenericEntry &nt_dist_kp;
    extern nt::GenericEntry &nt_dist_ki;
    extern nt::GenericEntry &nt_dist_kd;

    // Heading PID
    extern nt::GenericEntry &nt_heading;
    extern nt::GenericEntry &nt_heading_kp;
    extern nt::GenericEntry &nt_heading_ki;
    extern nt::GenericEntry &nt_heading_kd;

    // Turn PID
    extern nt::GenericEntry &nt_turn_kp;
    extern nt::GenericEntry &nt_turn_ki;
    extern nt::GenericEntry &nt_turn_kd;

    // Gyro
    extern nt::GenericEntry &nt_reset_gyro;

    // Save drivetrain parameters
    extern nt::GenericEntry &nt_save;
} // namespace DrivetrainUI

namespace ShooterUI {
    extern frc::ShuffleboardTab &nt_tab;

    // Write mode
    extern nt::GenericEntry &nt_write_mode;

    // sensors
    extern nt::GenericEntry &nt_ticks;
    extern nt::GenericEntry &nt_rpm;
    extern nt::GenericEntry &nt_output;
    extern nt::GenericEntry &nt_setpoint;
    extern nt::GenericEntry &nt_vel_err;
    extern nt::GenericEntry &nt_has_ball;

    // PID
    extern nt::GenericEntry &nt_shoot_kp;
    extern nt::GenericEntry &nt_shoot_ki;
    extern nt::GenericEntry &nt_shoot_kd;

    // Status
    extern nt::GenericEntry &nt_shooter_good;

    // Presets
    extern nt::GenericEntry &nt_normal_shot;
    extern nt::GenericEntry &nt_against_hub;
    extern nt::GenericEntry &nt_low_goal;
    extern nt::GenericEntry &nt_far_shot;
} // namespace ShooterUI

namespace SwerveDriveUI {
    // Get the tab
    extern frc::ShuffleboardTab &nt_tab;

    // Add all the defaults
    // Write mode
    extern nt::GenericEntry &nt_write_mode;

    // Encoder
    extern nt::GenericEntry &nt_left_avg;
    extern nt::GenericEntry &nt_right_avg;
    extern nt::GenericEntry &nt_avg_dist;

    // Distance PID
    extern nt::GenericEntry &nt_dist_kp;
    extern nt::GenericEntry &nt_dist_ki;
    extern nt::GenericEntry &nt_dist_kd;

    // Steer PID
    extern nt::GenericEntry &nt_steer_kp;
    extern nt::GenericEntry &nt_steer_ki;
    extern nt::GenericEntry &nt_steer_kd;

    // Gyro
    extern nt::GenericEntry &nt_heading;
    extern nt::GenericEntry &nt_reset_gyro;

    // Save drivetrain parameters
    extern nt::GenericEntry &nt_save;
} // namespace SwerveDriveUI