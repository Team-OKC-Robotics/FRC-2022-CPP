
#include "commands/shooter/SetTriggerCommand.h"

SetTriggerCommand::SetTriggerCommand(std::shared_ptr<Shooter> shooter,
                                     double power) {
    // Set everything.
    shooter_ = shooter;
    power_ = power;

    // Add the shooter as a requirement
    AddRequirements(shooter_.get());
}

void SetTriggerCommand::Execute() {
    VOKC_CHECK(shooter_ != nullptr);
    VOKC_CALL(shooter_->SetTrigger(power_));
}

bool SetTriggerCommand::IsFinished() {
    // Always end.
    return true;
}
