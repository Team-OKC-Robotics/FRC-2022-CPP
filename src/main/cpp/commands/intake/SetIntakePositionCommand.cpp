
#include "commands/intake/SetIntakePositionCommand.h"

SetIntakePositionCommand::SetIntakePositionCommand(
    std::shared_ptr<Intake> intake, bool is_extended) {
    // Set everything.
    intake_ = intake;
    is_extended_ = is_extended;

    if (intake_ != nullptr) {
        this->AddRequirements(intake_.get());
    }
}

void SetIntakePositionCommand::Execute() {
    VOKC_CHECK(intake_ != nullptr);
    VOKC_CALL(intake_->SetExtended(is_extended_));
}

bool SetIntakePositionCommand::IsFinished() {
    // Always end this command.
    return true;
}
