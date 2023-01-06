
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
#include "units/math.h"
#include "units/voltage.h"
#include "frc/controller/SimpleMotorFeedforward.h"

#include "Parameters.h"
#include "ui/UserInterface.h"
#include "Utils.h"
#include "io/SwerveDriveIO.h"

enum Location {
    LEFT_FRONT,
    LEFT_BACK,
    RIGHT_FRONT,
    RIGHT_BACK
};

class SwerveModule {
public:
    SwerveModule()
     : drive_pid(0.0, 0.001, 0.0001), steer_pid(0.0, 0.001, 0.0001), state(), pos(), trans(), location()/*, drive_ff(units::volt_t(0.0), units::volt_t(0.0) * units::second_t(1.0) / units::meter_t(1.0), units::volts_seconds_squared_per_meter(0.0)), steer_ff(0, 0, 0)*/ {};
    ~SwerveModule() {}

    bool Init(Location loc);

    bool GetSwerveModulePosition(frc::SwerveModulePosition *pos);
    bool GetSwerveModuleState(frc::SwerveModuleState *state);

    bool GetLocationOnRobot(frc::Translation2d *loc);
    
    bool SetDesiredState(frc::SwerveModuleState state);
    bool GetDriveOutput(double *output); // PID
    bool GetSteerOutput(double *output); // PID, optimize angle

    bool SetDrivePID(double kP, double kI, double kD);
    bool SetSteerPID(double kP, double kI, double kD);

    bool Update(double drive_enc, double steer_enc, double drive_vel, double steer_vel);
    bool Reset();

private:
    frc::PIDController drive_pid;
    frc::PIDController steer_pid;

    frc::SimpleMotorFeedforward<units::meters> drive_ff;
    frc::SimpleMotorFeedforward<units::degrees> steer_ff;

    frc::SwerveModuleState state;
    frc::SwerveModulePosition pos;

    frc::Translation2d trans;

    Location location;

    double drive_enc_vel;
    double steer_enc_vel;
};