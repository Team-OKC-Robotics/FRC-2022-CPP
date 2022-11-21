// Copyright (c) Team OKC Robotics

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <CANSparkMax.h>

#include "Utils.h"

class Drivetrain : public frc2::SubsystemBase
{
public:
    Drivetrain();

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
                     bool square_inputs = false);
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
    // Components (e.g. motor controllers and sensors) should generally be
    // declared private and exposed only through public methods.
};
