// Copyright (c) Team OKC Robotics

#pragma once

#define _USE_MATH_DEFINES
#include <math.h>
#include <memory>

#include <frc/controller/PIDController.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <wpi/array.h>
#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Pose2d.h"
#include <frc2/command/SubsystemBase.h>

#include "Parameters.h"
#include "ui/UserInterface.h"
#include "Utils.h"
#include "io/SwerveDriveIO.h"
#include "SwerveModule.h"

enum AutoStage {
    TURN_TO_GOAL,
    DRIVE_TO_GOAL,
    TURN_TO_FINAL_HEADING,
    FINISHED
};

class SwerveDrive : public frc2::SubsystemBase {
public:
    SwerveDrive(SwerveDriveSoftwareInterface *interface)
        : interface_(interface), left_front_module(), left_back_module(), right_front_module(), right_back_module(),
        swerve_kinematics(frc::Translation2d(units::meter_t(0.3), units::meter_t(0.3)), frc::Translation2d(units::meter_t(-0.3), units::meter_t(0.3)), frc::Translation2d(units::meter_t(0.3), units::meter_t(-0.3)), frc::Translation2d(units::meter_t(-0.3), units::meter_t(-0.3))),
        swerve_odometry(swerve_kinematics, frc::Rotation2d(), positions, frc::Pose2d()), position() {}
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
    bool DumbTeleOpDrive(const double &drive, const double &strafe, const double &turn);
    
    bool TranslateAuto(frc::Pose2d pos);
    bool TurnToHeading(frc::Pose2d pos);
    bool InitAuto(frc::Pose2d pos);
    bool TurnToGoal(frc::Pose2d pos);
    bool SetDrive(const double &power);
    bool DriveToGoal(frc::Pose2d pos);
    bool TurnToGoalHeading(frc::Pose2d pos);
    bool TranslateAutoLockHeading(frc::Pose2d pos);

    bool GetLeftDriveEncoderAverage(double *avg);
    bool GetRightDriveEncoderAverage(double *avg);
    bool GetDriveEncoderAverage(double *avg);
    bool GetLeftSteerEncoderAverage(double *avg);
    bool GetRightSteerEncoderAverage(double *avg);
    bool GetHeading(double *heading);

    bool AtSetpoint(bool *at);
    
    bool ResetDriveEncoders();
    bool ResetSteerEncoders();
    bool ResetPIDs();
    bool ResetGyro();

private:
    // Shuffleboard functions
    bool InitShuffleboard();
    bool UpdateShuffleboard();

    // software interface
    SwerveDriveSoftwareInterface *const interface_;

    // swerve modules
    SwerveModule left_front_module;
    SwerveModule left_back_module;
    SwerveModule right_front_module;
    SwerveModule right_back_module;

    // swerve module positions
    frc::Translation2d *left_front_loc;
    frc::Translation2d *left_back_loc;
    frc::Translation2d *right_front_loc;
    frc::Translation2d *right_back_loc;

    // swerve module states
    frc::SwerveModulePosition *left_front_pos;
    frc::SwerveModulePosition *left_back_pos;
    frc::SwerveModulePosition *right_front_pos;
    frc::SwerveModulePosition *right_back_pos;    

    // swerve module positions
    wpi::array<frc::SwerveModulePosition, 4> positions = {*left_front_pos, *left_back_pos, *right_front_pos, *right_back_pos};

    // kinematics
    frc::SwerveDriveKinematics<4> swerve_kinematics;

    // odometry
    frc::SwerveDriveOdometry<4> swerve_odometry;

    // position
    frc::Pose2d position;

    // Speed modifier (the joystick input is multiplied by this value)
    double speed_modifier_drive = 0.75;
    double speed_modifier_steer = 0.75;

    // max output
    double max_output_drive = 1;
    double max_output_steer = 1;

    // if the robot has reached the autonomous setpoint
    bool at_setpoint = false;

    double heading_to_goal;
    double distance_to_goal;
};
