
#pragma once

#include <memory>

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

#include "AHRS.h"
#include "Utils.h"


typedef struct swerve_drive_config_t {
    double max_output_drive;
    double open_loop_ramp_rate_drive;

    double max_output_steer;
    double open_loop_ramp_rate_steer;
} SwerveDriveConfig;

typedef struct swerve_drive_hardware_interface_t {
    // motors
    rev::CANSparkMax *const left_front_drive_motor;
    rev::CANSparkMax *const left_back_drive_motor;

    rev::CANSparkMax *const right_front_drive_motor;
    rev::CANSparkMax *const right_back_drive_motor;

    rev::CANSparkMax *const left_front_steer_motor;
    rev::CANSparkMax *const left_back_steer_motor;

    rev::CANSparkMax *const right_front_steer_motor;
    rev::CANSparkMax *const right_back_steer_motor;

    // AHRS
    AHRS *const ahrs;

} SwerveDriveHardwareInterface;

typedef struct swerve_drive_software_interface_t {
    // SW INPUTS
    // IMU yaw angle
    double imu_yaw;

    // Encoders
    double left_front_drive_motor_enc;
    double left_back_drive_motor_enc;

    double right_front_drive_motor_enc;
    double right_back_drive_motor_enc;

    double left_front_steer_motor_enc;
    double left_back_steer_motor_enc;

    double right_front_steer_motor_enc;
    double right_back_steer_motor_enc;


    // SW OUTPUTS
    // Configure swerve drive variables
    SwerveDriveConfig drive_config;
    bool update_config;

    // Input squaring
    bool square_inputs;

    // motor outputs
    double left_front_drive_motor_output;
    double left_back_drive_motor_output;

    double right_front_drive_motor_output;
    double right_back_drive_motor_output;

    double left_front_steer_motor_output;
    double left_back_steer_motor_output;

    double right_front_steer_motor_output;
    double right_back_steer_motor_output;

    // Reset flags
    bool reset_drive_encoders;
    bool reset_steer_encoders;
    bool reset_gyro;

} SwerveDriveSoftwareInterface;

class SwerveDriveIO : public frc2::SubsystemBase {
public:
    SwerveDriveIO(SwerveDriveHardwareInterface *hw_interface,
                 SwerveDriveSoftwareInterface *sw_interface)
        : hw_interface_(hw_interface), sw_interface_(sw_interface) {}

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;

    /**
     * Will be called periodically whenever the CommandScheduler runs during
     * simulation.
     */
    void SimulationPeriodic() override;

    bool ProcessIO();

private:
    bool UpdateDriveConfig(SwerveDriveConfig &config);
    bool ResetDriveEncoders();
    bool ResetSteerEncoders();

    SwerveDriveHardwareInterface *const hw_interface_;
    SwerveDriveSoftwareInterface *const sw_interface_;
};