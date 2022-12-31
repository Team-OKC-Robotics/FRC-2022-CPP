
#include "io/ShooterIO.h"

void ShooterIO::Periodic() {
    // Process all the inputs and outputs to/from high level software.
    VOKC_CALL(ProcessIO());
}

void ShooterIO::SimulationPeriodic() {
    // SimulationPeriodic
}

bool ShooterIO::ProcessIO() {
    OKC_CHECK(sw_interface_ != nullptr);
    OKC_CHECK(hw_interface_ != nullptr);

    // Update the shooter motor config if needed
    if (sw_interface_->update_shooter_config) {
        OKC_CALL(UpdateShooterConfig());
        sw_interface_->update_shooter_config = false;
    }

    // Update the trigger motor config if needed
    if (sw_interface_->update_trigger_config) {
        OKC_CALL(UpdateTriggerConfig());
        sw_interface_->update_trigger_config = false;
    }

    // Set motor powers
    OKC_CALL(SetMotorOutputs());

    // Read sensors
    OKC_CALL(ReadSensors());

    return true;
}

bool ShooterIO::UpdateShooterConfig() {
    OKC_CHECK(sw_interface_ != nullptr);
    OKC_CHECK(hw_interface_ != nullptr);
    OKC_CHECK(hw_interface_->shooter_motor != nullptr);

    // Reset to factory defaults.
    hw_interface_->shooter_motor->ConfigFactoryDefault();

    // Configure feedback sensor
    hw_interface_->shooter_motor->ConfigSelectedFeedbackSensor(
        ctre_mc::FeedbackDevice::IntegratedSensor, 0, 200);

    // Set inverted.
    hw_interface_->shooter_motor->SetInverted(
        sw_interface_->shooter_config.inverted);

    // Configure internal PID
    hw_interface_->shooter_motor->Config_kP(0, sw_interface_->shooter_config.Kp,
                                            200);
    hw_interface_->shooter_motor->Config_kI(0, sw_interface_->shooter_config.Ki,
                                            200);
    hw_interface_->shooter_motor->Config_kD(0, sw_interface_->shooter_config.Kd,
                                            200);

    // Neutral
    hw_interface_->shooter_motor->ConfigNeutralDeadband(
        sw_interface_->shooter_config.neutral_deadband);
    hw_interface_->shooter_motor->SetNeutralMode(
        sw_interface_->shooter_config.neutral_mode);

    // Open loop ramp rate
    hw_interface_->shooter_motor->ConfigOpenloopRamp(
        sw_interface_->shooter_config.open_loop_ramp_rate);

    return true;
}

bool ShooterIO::UpdateTriggerConfig() {
    OKC_CHECK(sw_interface_ != nullptr);
    OKC_CHECK(hw_interface_ != nullptr);
    OKC_CHECK(hw_interface_->trigger_motor != nullptr);

    hw_interface_->trigger_motor->SetSmartCurrentLimit(
        sw_interface_->trigger_config.smart_current_limit);

    return true;
}

bool ShooterIO::SetMotorOutputs() {
    OKC_CHECK(sw_interface_ != nullptr);
    OKC_CHECK(hw_interface_ != nullptr);
    OKC_CHECK(hw_interface_->trigger_motor != nullptr);
    OKC_CHECK(hw_interface_->shooter_motor != nullptr);

    // Stop shooter motor if the stop flag is set
    if (sw_interface_->stop_shooter) {
        hw_interface_->shooter_motor->Set(
            ctre_mc::TalonFXControlMode::PercentOutput, 0);
    } else {
        // Otherwise set whatever power is allocated from the subsystem.
        hw_interface_->shooter_motor->Set(
            ctre_mc::TalonFXControlMode::PercentOutput,
            sw_interface_->shooter_power);
    }

    // Set trigger motor appropriately
    hw_interface_->trigger_motor->Set(sw_interface_->trigger_power);

    return true;
}

bool ShooterIO::ReadSensors() {
    OKC_CHECK(sw_interface_ != nullptr);
    OKC_CHECK(hw_interface_ != nullptr);
    OKC_CHECK(hw_interface_->ball_detector != nullptr);

    // Detect the ball.
    sw_interface_->is_ball_detected = hw_interface_->ball_detector->Get();

    return true;
}