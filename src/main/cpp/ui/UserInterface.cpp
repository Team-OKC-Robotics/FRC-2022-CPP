#include "ui/UserInterface.h"

namespace DrivetrainUI {
    // Get the tab
    frc::ShuffleboardTab &nt_tab = frc::Shuffleboard::GetTab("Drivetrain");

    // Add all the defaults
    // Write mode
    nt::GenericEntry *const nt_write_mode =
        nt_tab.Add("Write Mode", false).GetEntry();

    // Encoder
    nt::GenericEntry *const nt_left_ticks =
        nt_tab.Add("left ticks", 0).GetEntry();
    nt::GenericEntry *const nt_right_ticks =
        nt_tab.Add("right ticks", 0).GetEntry();
    nt::GenericEntry *const nt_total_ticks =
        nt_tab.Add("total ticks", 0).GetEntry();
    nt::GenericEntry *const nt_dist_error =
        nt_tab.Add("distance error", 0).GetEntry();

    // Distance PID
    nt::GenericEntry *const nt_dist_kp =
        nt_tab.Add("Distance kP", 0).GetEntry();
    nt::GenericEntry *const nt_dist_ki =
        nt_tab.Add("Distance kI", 0).GetEntry();
    nt::GenericEntry *const nt_dist_kd =
        nt_tab.Add("Distance kD", 0).GetEntry();

    // Heading PID
    nt::GenericEntry *const nt_heading = nt_tab.Add("Heading", 0).GetEntry();
    nt::GenericEntry *const nt_heading_kp =
        nt_tab.Add("Heading kP", 0).GetEntry();
    nt::GenericEntry *const nt_heading_ki =
        nt_tab.Add("Heading kI", 0).GetEntry();
    nt::GenericEntry *const nt_heading_kd =
        nt_tab.Add("Heading kD", 0).GetEntry();

    // Turn PID
    nt::GenericEntry *const nt_turn_kp = nt_tab.Add("Turn kP", 0).GetEntry();
    nt::GenericEntry *const nt_turn_ki = nt_tab.Add("Turn kI", 0).GetEntry();
    nt::GenericEntry *const nt_turn_kd = nt_tab.Add("Turn kD", 0).GetEntry();

    // Gyro
    nt::GenericEntry *const nt_reset_gyro =
        nt_tab.Add("Reset Gyro", false).GetEntry();

    // Save parameters button
    nt::GenericEntry *const nt_save = nt_tab.Add("Save", false).GetEntry();
} // namespace DrivetrainUI

namespace ShooterUI {
    // Get the tab
    frc::ShuffleboardTab &nt_tab = frc::Shuffleboard::GetTab("Shooter");

    // Write mode
    nt::GenericEntry *const nt_write_mode =
        nt_tab.Add("Write Mode", false).GetEntry();

    // sensors
    nt::GenericEntry *const nt_ticks =
        nt_tab.Add("Shooter Ticks", 0).GetEntry();
    nt::GenericEntry *const nt_rpm = nt_tab.Add("Shooter RPM", 0).GetEntry();
    nt::GenericEntry *const nt_output =
        nt_tab.Add("Shooter Output", 0).GetEntry();
    nt::GenericEntry *const nt_setpoint = nt_tab.Add("Setpoint", 0).GetEntry();
    nt::GenericEntry *const nt_vel_err =
        nt_tab.Add("Velocity Error", 0).GetEntry();
    nt::GenericEntry *const nt_has_ball =
        nt_tab.Add("Has Ball?", false).GetEntry();

    // PID
    nt::GenericEntry *const nt_shoot_kp =
        nt_tab.Add("Shooter kP", 0).GetEntry();
    nt::GenericEntry *const nt_shoot_ki =
        nt_tab.Add("Shooter kI", 0).GetEntry();
    nt::GenericEntry *const nt_shoot_kd =
        nt_tab.Add("Shooter kD", 0).GetEntry();

    // Status
    nt::GenericEntry *const nt_shooter_good =
        nt_tab.Add("Shooter Good", false).GetEntry();

    // Presets
    nt::GenericEntry *const nt_normal_shot =
        nt_tab.Add("Normal Shot Preset", 0).GetEntry();
    nt::GenericEntry *const nt_against_hub =
        nt_tab.Add("Against Hub Preset", 0).GetEntry();
    nt::GenericEntry *const nt_low_goal =
        nt_tab.Add("Low Goal Preset", 0).GetEntry();
    nt::GenericEntry *const nt_far_shot =
        nt_tab.Add("Far Shot Preset", 0).GetEntry();

} // namespace ShooterUI

namespace SwerveDriveUI {
    // Get the tab
    frc::ShuffleboardTab &nt_tab = frc::Shuffleboard::GetTab("SwerveDrive");

    // Add all the defaults
    // Write mode
    nt::GenericEntry *const nt_write_mode =
        nt_tab.Add("Write Mode", false).GetEntry();

    // Encoder
    nt::GenericEntry *const nt_left_avg = nt_tab.Add("left avg dist", 0).GetEntry();
    nt::GenericEntry *const nt_right_avg = nt_tab.Add("right avg dist", 0).GetEntry();
    nt::GenericEntry *const nt_avg_dist = nt_tab.Add("avg dist", 0).GetEntry();

    // Distance PID
    nt::GenericEntry *const nt_dist_kp = nt_tab.Add("Distance kP", 0).GetEntry();
    nt::GenericEntry *const nt_dist_ki = nt_tab.Add("Distance kI", 0).GetEntry();
    nt::GenericEntry *const nt_dist_kd = nt_tab.Add("Distance kD", 0).GetEntry();

    // Steer PID
    nt::GenericEntry *const nt_steer_kp = nt_tab.Add("Steer kP", 0).GetEntry();
    nt::GenericEntry *const nt_steer_ki = nt_tab.Add("Steer kI", 0).GetEntry();
    nt::GenericEntry *const nt_steer_kd = nt_tab.Add("Steer kD", 0).GetEntry();

    // Gyro
    nt::GenericEntry *const nt_heading = nt_tab.Add("Heading", 0).GetEntry();
    nt::GenericEntry *const nt_reset_gyro =
        nt_tab.Add("Reset Gyro", false).GetEntry();

    // Save parameters button
    nt::GenericEntry *const nt_save = nt_tab.Add("Save", false).GetEntry();
} // namespace SwerveDriveUI