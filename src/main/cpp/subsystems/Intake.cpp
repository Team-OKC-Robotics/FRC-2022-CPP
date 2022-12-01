
#include "subsystems/Intake.h"
#include "Utils.h"

bool Intake::Init() {
    // TODO: Set PID gains.

    OKC_CALL(SetOpenLoopRamp(0.5));

    // Set PID tolerances
    intake_pid.SetTolerance(0, 0); //TODO change to actual number
    
    // TODO: shuffleboard.

    // Reset everything
    OKC_CALL(ResetPositionEncoder());
    OKC_CALL(ResetPositionPID());

    return true;
}

void Intake::Periodic() {
    // TODO: implement once network tables and constants are set up
    // It looks like this is just setting PID gains over and over.

    // if the subsystem has been commanded to move in a direction (otherwise remain coasting and freely moving)
    if (direction != 0) {
        // if the switch is pressed
        if (!this->interface_->deployed_limit_switch_val) { // reverse logic because switches are normally closed (so pressed=false)
            // set the encoder to the value it is supposed to be when pressed
            this->interface_->set_encoder_to_val = true;
            this->interface_->encoder_val_to_set = this->interface_->intake_config.EXTENDED;
        }

        // if we are deploying and the intake is deployed (limit switch being pressed)
        if (direction == 1 && !this->interface_->deployed_limit_switch_val) {
            this->interface_->intake_position_power = 0;
        } else { // otherwise keep going
            this->interface_->intake_position_power =  intake_pid.Calculate(this->interface_->intake_position_encoder_val);
            
            //FIXME I think there's supposed to be an OKC_CALL() around this, but it was giving me errors, so I don't have it
            TeamOKC::Clamp(this->interface_->max_output_retract, this->interface_->max_output_deploy, &this->interface_->intake_position_power);
        }
    }
}

void Intake::SimulationPeriodic() {
    // SimulationPeriodic
}

bool Intake::ResetPositionPID() {
    this->intake_pid.Reset();

    return true;
}

bool Intake::ResetPositionEncoder() {
    this->interface_->reset_encoders = true;

    return true;
}

bool Intake::SetIntakePower(const double &power) {
    this->interface_->intake_power = power;

    return true;
}

bool Intake::SetIndexerPower(const double &power) {
    this->interface_->indexer_power = power;

    return true;
}

bool Intake::SetExtended(const bool &extended) {
    if (extended) {
        intake_pid.SetSetpoint(this->interface_->intake_config.EXTENDED);
        direction = 1;
    } else {
        intake_pid.SetSetpoint(this->interface_->intake_config.RETRACTED);
        direction = -1;
    }

    return true;
}

// this method seems useful but we don't end up using it in the Java version so it might get cut
// leaving here (untouched except for commented out) for now
/**
bool Intake::IsExtended(bool *extended) { //TODO that's not the right code/logic to cover all cases and would be a bad idea anyways but this is a placeholder
    *extended = this->interface_->deployed_limit_switch_val;
    
    return true;
}
*/