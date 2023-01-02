
#include "commands/intake/SetIntakeCommand.h"

SetIntakeCommand::SetIntakeCommand(std::shared_ptr<Intake> intake,
                                   double power) {
    // Set everything.
    intake_ = intake;
    power_ = power;

    if (intake_ != nullptr) {
        this->AddRequirements(intake_.get());
    }
}

void SetIntakeCommand::Execute() {
    VOKC_CHECK(intake_ != nullptr);
    VOKC_CALL(intake_->SetIntakePower(power_));
}

bool SetIntakeCommand::IsFinished() {
    // Always end this command.
    return true;
}
