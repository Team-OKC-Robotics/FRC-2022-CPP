// Copyright (c) Team OKC Robotics

#pragma once

#define _USE_MATH_DEFINES
#include <math.h>
#include <memory>

#include <frc/controller/PIDController.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <wpi/array.h>
#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Translation2d.h"
#include <frc2/command/SubsystemBase.h>
#include "units/length.h"
#include "units/velocity.h"

#include "Parameters.h"
#include "ui/UserInterface.h"
#include "Utils.h"
#include "io/SwerveDriveIO.h"

class SwerveDrive : public frc2::SubsystemBase {
public:
    // TODO: put the actual constants in for the PID gains.
    SwerveDrive(SwerveDriveSoftwareInterface *interface)
        : interface_(interface), swerve_kinematics(frc::Translation2d(units::meter_t(0.3), units::meter_t(0.3)), frc::Translation2d(units::meter_t(-0.3), units::meter_t(0.3)), frc::Translation2d(units::meter_t(0.3), units::meter_t(-0.3)), frc::Translation2d(units::meter_t(-0.3), units::meter_t(-0.3))),
        left_front_module(), left_back_module(), right_front_module(), right_back_module(),
        swerve_odometry(swerve_kinematics, frc::Rotation2d(), modules, frc::Pose2d()), 
        left_front_drive_pid(0, 0.001, 0.0001), left_back_drive_pid(0, 0.001, 0.0001), right_front_drive_pid(0, 0.001, 0.0001), right_back_drive_pid(0, 0.001, 0.0001), 
        left_front_steer_pid(0, 0.001, 0.0001), left_back_steer_pid(0, 0.001, 0.0001), right_front_steer_pid(0, 0.001, 0.0001), right_back_steer_pid(0, 0.001, 0.0001) {}
    ~SwerveDrive() {}

    bool Init();

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;

    /**
     * Will be called periodically whenever the CommandScheduler runs during
     * simulation.
     */
    void SimulationPeriodic() override;

    bool SetSpeedModifierDrive(const double &speed_mod);
    bool SetSpeedModifierSteer(const double &speed_mod);
    bool SetOpenLoopRampDrive(const double &open_loop_ramp);
    bool SetOpenLoopRampSteer(const double &open_loop_ramp);
    bool SetMaxOutputDrive(const double &max_output);
    bool SetMaxOutputSteer(const double &max_output);

    bool TeleOpDrive(const double &drive, const double &strafe, const double &turn);
    
    bool TranslateAuto(const double &x, const double &y, const double &rot);
    bool TurnToHeading(const double &heading);
    bool GetHeading(double *heading);
    
    bool ResetDriveEncoders();
    bool ResetSteerEncoders();
    bool ResetGyro();

private:
    // Shuffleboard functions
    bool InitShuffleboard();
    bool UpdateShuffleboard();

    // software interface
    SwerveDriveSoftwareInterface *const interface_;


    // swerve modules states
    frc::SwerveModuleState left_front_module;
    frc::SwerveModuleState left_back_module;
    frc::SwerveModuleState right_front_module;
    frc::SwerveModuleState right_back_module;

    // swerve module positions
    frc::SwerveModulePosition left_front_pos;
    frc::SwerveModulePosition left_back_pos;
    frc::SwerveModulePosition right_front_pos;
    frc::SwerveModulePosition right_back_pos;

    // swerve module list
    wpi::array<frc::SwerveModuleState, 4> outputs = {left_front_module, left_back_module, right_front_module, right_back_module};
    wpi::array<frc::SwerveModulePosition, 4> modules = {left_front_pos, left_back_pos, right_front_pos, right_back_pos};

    // kinematics
    frc::SwerveDriveKinematics<4> swerve_kinematics;

    // odometry
    frc::SwerveDriveOdometry<4> swerve_odometry;

    // PID Controllers
    frc::PIDController left_front_drive_pid;
    frc::PIDController left_back_drive_pid;
    frc::PIDController right_front_drive_pid;
    frc::PIDController right_back_drive_pid;

    frc::PIDController left_front_steer_pid;
    frc::PIDController left_back_steer_pid;
    frc::PIDController right_front_steer_pid;
    frc::PIDController right_back_steer_pid;


    // Speed modifier (the joystick input is multiplied by this value)
    double speed_modifier_drive = 0.75;
    double speed_modifier_steer = 0.75;

    // Open loop ramp rate
    double open_loop_ramp_ = 0.5;
};