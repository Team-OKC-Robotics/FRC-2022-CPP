
#include "subsystems/Intake.h"
#include "Utils.h"

bool Intake::Init() {
    // TODO: Set PID gains.

    // update the intake config
    this->interface_->intake_config.open_loop_ramp_rate = open_loop_ramp_;
    this->interface_->intake_config.EXTENDED = 43.75; // TODO: replace with a reference to Constants. as-is the number is copypastad from there anyways, but still
    this->interface_->intake_config.RETRACTED = 0;
    this->interface_->intake_config.max_output_deploy = 1;
    this->interface_->intake_config.max_output_retract = -1;
    this->interface_->intake_config.max_indexer_current = 20; // limit to 20 amps

    this->interface_->update_config = true;

    // Set PID tolerances
    //intake_pid.SetTolerance(0.5, 0.5); //TODO change to actual number
    
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
            TeamOKC::Clamp(this->interface_->intake_config.max_output_retract, this->interface_->intake_config.max_output_deploy, &this->interface_->intake_position_power);
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

/**
 * A method to get the 'direction' variable of the intake subsystem
 * This should only be used to verify proper functioning of the intake subsystem
 */
int Intake::GetDirection() {
    return direction;
}

/**
 * A method to get the setpoint of the intake position PID controller
 * Mainly for unit tests right now
 */
double Intake::GetSetpoint() {
    return intake_pid.GetSetpoint();
}

/**
 * Returns if the intake is extended or not
 * aka if the limit switch is being pressed, although there are times when it *is* extended
 * and the switch is not pressed, because it bounces and the switches break a lot. That is not
 * an important edge case, however, because nothing relies on this method other than unit tests.
 */
bool Intake::IsExtended() {
    // inverse logic, so false is pressed, and true is released
    return this->interface_->deployed_limit_switch_val == false;
}

/**
 * Returns if the intake is retracted or not
 * probably wildely inaccurate because it's based on the encoder,
 * but this is mainly here for unit testing purposes. In the future,
 * might be better to have some kind of margin of error like
 * abs(encoder_val) < 2 or something like that.
 */
bool Intake::IsRetracted() {
    return abs(this->interface_->intake_position_encoder_val) < 0.1; // stupid weird floaty numbers makin' me use abs()
}