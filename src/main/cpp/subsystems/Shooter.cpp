
#include "subsystems/Shooter.h"
#include "Utils.h"

bool Shooter::Init() {
    OKC_CHECK(interface_ != nullptr);

    // Set PID gains.
    double shoot_p = RobotParams::GetParam("shooter.shoot_pid.Kp", 0);
    double shoot_i = RobotParams::GetParam("shooter.shoot_pid.Ki", 0);
    double shoot_d = RobotParams::GetParam("shooter.shoot_pid.Kp", 0);
    shooter_pid_.SetPID(shoot_p, shoot_i, shoot_d);
    shooter_pid_.SetTolerance(100, 100);

    // Configure shooter motor
    interface_->shooter_config.inverted =
        ctre_mc::InvertType::InvertMotorOutput;
    interface_->shooter_config.Kp = shoot_p;
    interface_->shooter_config.Ki = shoot_i;
    interface_->shooter_config.Kd = shoot_d;
    interface_->shooter_config.open_loop_ramp_rate = 0;
    interface_->shooter_config.neutral_mode = ctre_mc::Coast;
    interface_->shooter_config.neutral_deadband = 0;
    interface_->shooter_config.sensor_pos = 0;
    interface_->update_shooter_config = true;

    // Configure trigger motor
    interface_->trigger_config.smart_current_limit = 30;
    interface_->update_trigger_config = true;

    // Initialize logging.
    rpm_log_ = std::make_unique<wpi::log::DoubleLogEntry>(TeamOKC::log,
                                                          "/shooter/rpm");
    setpoint_log_ = std::make_unique<wpi::log::DoubleLogEntry>(
        TeamOKC::log, "/shooter/setpoint");
    output_log_ = std::make_unique<wpi::log::DoubleLogEntry>(TeamOKC::log,
                                                             "/shooter/output");
    calculated_log_ = std::make_unique<wpi::log::DoubleLogEntry>(
        TeamOKC::log, "/shooter/pid-calculate");

    return true;
}

void Shooter::Periodic() {
    VOKC_CHECK(interface_ != nullptr);

    // Update shuffleboard
    VOKC_CALL(UpdateShuffleboard());

    // Update RPM log.
    rpm_log_->Append(interface_->shooter_rpm);
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

    // Update the PID calculation log.
    calculated_log_->Append(power);

    // Clamp power to be between 0 and 1.
    OKC_CALL(TeamOKC::Clamp(0, 1, &power));

    // Update logs.
    setpoint_log_->Append(rpm);
    output_log_->Append(power);

    // Update shuffleboard setpoint.
    ShooterUI::nt_setpoint->SetDouble(rpm);

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

    double rpm = 0.0;

    switch (preset) {
    case ShooterPreset::NORMAL_SHOT:
        rpm = RobotParams::GetParam("shooter.normal_shot", 0);
        OKC_CALL(SetShooter(rpm));
        break;
    case ShooterPreset::AGAINST_HUB:
        rpm = RobotParams::GetParam("shooter.against_hub", 0);
        OKC_CALL(SetShooter(rpm));
        break;
    case ShooterPreset::LOW_GOAL:
        rpm = RobotParams::GetParam("shooter.low_goal", 0);
        OKC_CALL(SetShooter(rpm));
        break;
    case ShooterPreset::FAR_SHOT:
        rpm = RobotParams::GetParam("shooter.far_shot", 0);
        OKC_CALL(SetShooter(rpm));
        break;
    default:
        return false;
    }

    return true;
}

bool Shooter::IsAtShooterSetpoint(bool *at_setpoint) {
    OKC_CHECK(at_setpoint != nullptr);

    bool non_zero_setpoint = shooter_pid_.GetSetpoint() > 0;
    bool setpoint_achieved = shooter_pid_.AtSetpoint();
    *at_setpoint = non_zero_setpoint && setpoint_achieved;

    return true;
}

bool Shooter::SetTrigger(const double &power) {
    OKC_CHECK(interface_ != nullptr);

    if (interface_->is_ball_detected) {
        // If the ball is detected, only allow the ball to move backwards.
        if (power <= 0) {
            interface_->trigger_power = power;
        } else {
            interface_->trigger_power = 0.0;
        }

    } else {
        // Run the trigger as much as you want if no ball detected.
        interface_->trigger_power = power;
    }

    return true;
}

bool Shooter::Feed(const double &power) {
    OKC_CHECK(interface_ != nullptr);

    // Ignore ball detection, this is for when we want to shoot.
    interface_->trigger_power = power;

    return true;
}

bool Shooter::InitShuffleboard() {
    // Get parameters
    double shoot_p = RobotParams::GetParam("shooter.shoot_pid.Kp", 0);
    double shoot_i = RobotParams::GetParam("shooter.shoot_pid.Ki", 0);
    double shoot_d = RobotParams::GetParam("shooter.shoot_pid.Kp", 0);

    ShooterUI::nt_shoot_kp->SetDouble(shoot_p);
    ShooterUI::nt_shoot_ki->SetDouble(shoot_i);
    ShooterUI::nt_shoot_kd->SetDouble(shoot_d);

    return true;
}

bool Shooter::UpdateShuffleboard() {
    OKC_CHECK(interface_ != nullptr);
    // If competition mode is false, allow modification of the PID gains
    bool is_competition = RobotParams::GetParam("competition", false);
    if (!is_competition) {
        // For some reason we don't update the shooter displays if it's a
        // competition
        ShooterUI::nt_rpm->SetDouble(interface_->shooter_rpm);

        bool at_setpoint = false;
        OKC_CALL(IsAtShooterSetpoint(&at_setpoint));
        ShooterUI::nt_shooter_good->SetBoolean(at_setpoint);

        ShooterUI::nt_vel_err->SetDouble(interface_->velocity_error);
        ShooterUI::nt_ticks->SetDouble(interface_->ticks);
        ShooterUI::nt_output->SetDouble(interface_->shooter_output_pct);

        if (ShooterUI::nt_write_mode->GetBoolean(false)) {
            // Get the current PID parameter values.
            double shoot_p = RobotParams::GetParam("shooter.shoot_pid.Kp", 0);
            double shoot_i = RobotParams::GetParam("shooter.shoot_pid.Ki", 0);
            double shoot_d = RobotParams::GetParam("shooter.shoot_pid.Kp", 0);

            // Get the values from shuffleboard.
            shoot_p = ShooterUI::nt_shoot_kp->GetDouble(shoot_p);
            shoot_i = ShooterUI::nt_shoot_ki->GetDouble(shoot_i);
            shoot_d = ShooterUI::nt_shoot_kd->GetDouble(shoot_d);

            // Set the PID gains.
            shooter_pid_.SetPID(shoot_p, shoot_i, shoot_d);
        }
    }

    return true;
}
