// Copyright (c) Team OKC Robotics

#pragma once

#include <memory>

#include <frc/controller/PIDController.h>
#include <frc2/command/SubsystemBase.h>

#include <frc/DataLogManager.h>
#include <wpi/DataLog.h>

#include "Parameters.h"
#include "Utils.h"
#include "io/ShooterIO.h"
#include "ui/UserInterface.h"


enum ShooterPreset {
    NORMAL_SHOT,
    AGAINST_HUB,
    LOW_GOAL,
    FAR_SHOT
};

class Shooter : public frc2::SubsystemBase {
public:
    // TODO: put the actual constants in for the PID gains.
    Shooter(ShooterSoftwareInterface *interface)
        : interface_(interface), shooter_pid_(1.0, 0.0, 0.0) {}
    ~Shooter() {}

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

    bool ResetPower();

    bool SetShooter(const double &rpm);

    bool StopShooter();

    bool SetShooterPreset(ShooterPreset preset);

    bool IsAtShooterSetpoint(bool *at_setpoint);

    bool SetTrigger(const double &power);

    bool Feed(const double &power);

private:
    bool InitShuffleboard();
    bool UpdateShuffleboard();

    ShooterSoftwareInterface *const interface_;

    // Controllers
    frc2::PIDController shooter_pid_;

    // Logging
    std::unique_ptr<wpi::log::DoubleLogEntry> rpm_log_;
    std::unique_ptr<wpi::log::DoubleLogEntry> setpoint_log_;
    std::unique_ptr<wpi::log::DoubleLogEntry> output_log_;
    std::unique_ptr<wpi::log::DoubleLogEntry> calculated_log_;
};
