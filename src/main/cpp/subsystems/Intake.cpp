
#include "subsystems/Intake.h"

bool Intake::Init() {
    // TODO: Set PID gains.

    //OKC_CALL(SetOpenLoopRamp(0.01));

    // Set PID tolerances
    intake_pid.SetTolerance(0); //TODO change to actual number
    
    // TODO: shuffleboard.

    // Reset everything
    OKC_CALL(ResetPositionEncoder());
    OKC_CALL(ResetPositionPID());

    return true;
}

void Intake::Periodic() {
    // TODO: implement once network tables and constants are set up
    // It looks like this is just setting PID gains over and over.

    // also the actual intake up/down PID logic stuff that is bascially the core of the subsystem
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
    //TODO actually implement code lol

    return true;
}

bool Intake::IsExtended(bool *extended) { //TODO that's not the right code/logic to cover all cases and would be a bad idea anyways but this is a placeholder
    *extended = this->interface_->deployed_limit_switch_val;
    
    return true;
}