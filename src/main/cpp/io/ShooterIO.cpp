
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
}