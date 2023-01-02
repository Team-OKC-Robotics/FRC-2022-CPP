
#include "commands/intake/ResetIntakeEncoderCommand.h"

ResetIntakeEncoderCommand::ResetIntakeEncoderCommand(
    std::shared_ptr<Intake> intake) {
    // Set everything.
    intake_ = intake;

    if (intake_ != nullptr) {
        this->AddRequirements(intake_.get());
    }
}

void ResetIntakeEncoderCommand::Execute() {
    VOKC_CHECK(intake_ != nullptr);
    VOKC_CALL(intake_->ResetPositionEncoder());
}

bool ResetIntakeEncoderCommand::IsFinished() {
    // Always end this command.
    return true;
}
