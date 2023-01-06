
#include "commands/intake/SetIndexerCommand.h"

SetIndexerCommand::SetIndexerCommand(std::shared_ptr<Intake> intake,
                                     double power) {
    // Set everything.
    intake_ = intake;
    power_ = power;

    if (intake_ != nullptr) {
        this->AddRequirements(intake_.get());
    }
}

void SetIndexerCommand::Execute() {
    VOKC_CHECK(intake_ != nullptr);
    VOKC_CALL(intake_->SetIndexerPower(power_));
}

bool SetIndexerCommand::IsFinished() {
    // Always end this command.
    return true;
}
