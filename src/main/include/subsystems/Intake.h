// Copyright (c) Team OKC Robotics

#pragma once

#include <memory>

#include <frc/controller/PIDController.h>
#include <frc2/command/SubsystemBase.h>

#include "Utils.h"
#include "io/IntakeIO.h"

class Intake : public frc2::SubsystemBase {
public:
    // TODO: put the actual constants in for the PID gains.
    Intake(IntakeSoftwareInterface *interface)
        : interface_(interface), intake_pid(1.0, 0.0, 0.0) {}
    ~Intake() {}

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

    bool ResetPositionEncoder();
    bool ResetPositionPID();
    
    bool SetIntakePower(const double &power);
    bool SetIndexerPower(const double &power);
    bool SetExtended(const bool &extended);

    bool IsExtended(bool *extended);

private:
    IntakeSoftwareInterface *const interface_;

    // Controllers
    frc2::PIDController intake_pid;

    // Open loop ramp rate
    double open_loop_ramp_ = 0.5;
};
