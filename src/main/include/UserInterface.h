
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
} // namespace DrivetrainUI