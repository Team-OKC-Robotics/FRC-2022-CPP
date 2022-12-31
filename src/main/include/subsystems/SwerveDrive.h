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
#include <frc2/command/SubsystemBase.h>

#include "Parameters.h"
#include "UserInterface.h"
#include "Utils.h"
#include "io/SwerveDriveIO.h"

class SwerveDrive : public frc2::SubsystemBase {
public:
    // TODO: put the actual constants in for the PID gains.
    SwerveDrive(SwerveDriveSoftwareInterface *interface)
        : interface_(interface), swerve_kinematics(frc::Translation2d(/*TODO parameter loading, etc*/), frc::Translation2d(/*TODO*/), frc::Translation2d(/*TODO*/), frc::Translation2d(/*TODO*/),
        swerve_odometry(&swerve_kinematics, 0, /*TODO*/),
        left_front_module, left_back_module, right_front_module, right_back_module) {}
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

    // kinematics
    frc::SwerveDriveKinematics<4> swerve_kinematics;

    // odometry
    frc::SwerveDriveOdometry<4> swerve_odometry;

    // swerve modules
    frc::SwerveModuleState left_front_module;
    frc::SwerveModuleState left_back_module;
    frc::SwerveModuleState right_front_module;
    frc::SwerveModuleState right_back_module;

    // Speed modifier (the joystick input is multiplied by this value)
    double speed_modifier_drive = 0.75;
    double speed_modifier_steer = 0.75;

    // Open loop ramp rate
    double open_loop_ramp_ = 0.5;
};
