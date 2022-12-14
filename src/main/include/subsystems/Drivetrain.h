// Copyright (c) Team OKC Robotics

#pragma once

#define _USE_MATH_DEFINES
#include <math.h>
#include <memory>

#include <frc/controller/PIDController.h>
#include <frc2/command/SubsystemBase.h>

#include "Parameters.h"
#include "Utils.h"
#include "io/DrivetrainIO.h"
#include "ui/UserInterface.h"


class Drivetrain : public frc2::SubsystemBase {
public:
    // TODO: put the actual constants in for the PID gains.
    Drivetrain(DrivetrainSoftwareInterface *interface)
        : interface_(interface), dist_pid_(1.0, 0.0, 0.0),
          heading_pid_(1.0, 0.0, 0.0), turn_pid_(1.0, 0.0, 0.0) {}
    ~Drivetrain() {}

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

    bool SetSpeedModifier(const double &speed_mod);
    bool SetOpenLoopRamp(const double &open_loop_ramp);
    bool CurvatureDrive(const double &speed, const double &turn,
                        const bool turn_in_place = false);
    bool TankDrive(const double &left_speed, const double &right_speed);
    bool ArcadeDrive(const double &speed, const double &turn,
                     bool square_inputs = true);
    bool ArcadeDriveAuto(const double &speed, const double &turn,
                         bool square_inputs = false);
    bool DriveDistance(const double &distance);
    bool DriveOnHeading(const double &speed, const double &distance);

    bool TurnToHeading(const double &heading);
    bool SetHeading();
    bool GetEncoderAverage(double *avg);
    bool GetLeftEncoderAverage(double *left_avg);
    bool GetRightEncoderAverage(double *right_avg);
    bool ResetEncoders();
    bool GetInches(const double &rotations, double *inches);
    bool IsAtDistanceSetpoint(bool *at_setpoint);
    bool IsAtHeadingSetpoint(bool *at_setpoint);
    bool IsAtTurnSetpoint(bool *at_setpoint);
    bool ResetDistancePID();
    bool ResetHeadingPID();
    bool ResetTurnPID();

    bool GetHeading(double *heading);
    bool ResetGyro();
    bool SetMaxOutput(const double &max_output);

private:
    // Shuffleboard funcitons
    bool InitShuffleboard();
    bool UpdateShuffleboard();

    DrivetrainSoftwareInterface *const interface_;

    // Controllers
    frc2::PIDController dist_pid_;
    frc2::PIDController heading_pid_;
    frc2::PIDController turn_pid_;

    // Speed modifier (the joystick input is multiplied by this value)
    double speed_modifier_ = 0.75;

    // Open loop ramp rate
    double open_loop_ramp_ = 0.5;
};
