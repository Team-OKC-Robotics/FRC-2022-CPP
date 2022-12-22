
#include "subsystems/Shooter.h"
#include "Utils.h"

bool Shooter::Init() {
    // TODO: Set PID gains.
}

void Shooter::Periodic() {
    // TODO: implement once network tables and constants are set up
}

void Shooter::SimulationPeriodic() {
    // SimulationPeriodic
}

bool Shooter::ResetPower() {
    OKC_CHECK(interface_ != nullptr);
    interface_->shooter_power = 0;

    return true;
}

bool Shooter::SetShooter(const double &rpm) {
    OKC_CHECK(interface_ != nullptr);

    double pid_out = -1 * shooter_pid_.Calculate(interface_->shooter_rpm, rpm);
    double power = interface_->shooter_power + pid_out;

    // Clamp power to be between 0 and 1.
    OKC_CALL(TeamOKC::Clamp(0, 1, &power));

    // Output power.
    interface_->shooter_power = power;

    return true;
}

bool Shooter::StopShooter() {
    OKC_CHECK(interface_ != nullptr);

    interface_->stop_shooter = true;

    return true;
}

bool Shooter::SetShooterPreset(ShooterPreset preset) {
    OKC_CHECK(interface_ != nullptr);

    switch (preset) {
    case ShooterPreset::NORMAL_SHOT:
        // OKC_CALL(SetShooter())
        break;
    case ShooterPreset::AGAINST_HUB:
        break;
    case ShooterPreset::LOW_GOAL:
        break;
    case ShooterPreset::FAR_SHOT:
        break;
    default:
        return false;
    }

    return true;
}

bool Shooter::IsAtShooterSetpoint(bool *at_setpoint) {}
bool Shooter::SetTrigger(const double &power) {}
bool Shooter::Feed(const double &power) {}