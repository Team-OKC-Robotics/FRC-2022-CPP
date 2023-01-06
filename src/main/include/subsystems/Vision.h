// Copyright (c) Team OKC Robotics

#pragma once

#include <memory>

#include <frc/controller/PIDController.h>
#include <frc2/command/SubsystemBase.h>

#include "Parameters.h"
#include "Utils.h"
#include "io/VisionIO.h"

class Vision : public frc2::SubsystemBase {
public:
    // TODO: put the actual constants in for the PID gains.
    Vision(VisionSoftwareInterface *interface)
        : interface_(interface), vision_pid_(1, 0, 0) {}
    ~Vision() {}

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

    bool ResetPID();
    bool GetOutput(double *output);
    bool IsAtVisionSetpoint(bool *at_setpoint);
    bool SetLEDs(bool &is_on);
    bool GetDistance(double *distance);

private:
    VisionSoftwareInterface *const interface_;

    // PID controller for vision alignment
    frc::PIDController vision_pid_;
};
