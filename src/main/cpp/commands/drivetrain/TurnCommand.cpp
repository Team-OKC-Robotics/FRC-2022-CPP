
#include "commands/drivetrain/TurnCommand.h"

TurnCommand::TurnCommand(std::shared_ptr<Drivetrain> drivetrain, double angle,
                         double max_output) {
    // Set everything.
    drivetrain_ = drivetrain;
    angle_ = angle;
    max_output_ = max_output;

    if (drivetrain_ != nullptr) {
        this->AddRequirements(drivetrain_.get());
    }
}

void TurnCommand::Initialize() {
    VOKC_CHECK(drivetrain_ != nullptr);

    VOKC_CALL(drivetrain_->ResetTurnPID());
    VOKC_CALL(drivetrain_->ResetEncoders());
    VOKC_CALL(drivetrain_->ResetGyro());
    VOKC_CALL(drivetrain_->SetMaxOutput(max_output_));
}

void TurnCommand::Execute() {
    VOKC_CHECK(drivetrain_ != nullptr);

    VOKC_CALL(drivetrain_->TurnToHeading(angle_));
}

bool TurnCommand::IsFinished() {
    if (drivetrain_ == nullptr) {
        return true;
    }

    bool turn_achieved = false;
    if (drivetrain_->IsAtTurnSetpoint(&turn_achieved)) {
        return turn_achieved;
    }

    // If the check of the setpoint failed, end the command.
    return true;
}
