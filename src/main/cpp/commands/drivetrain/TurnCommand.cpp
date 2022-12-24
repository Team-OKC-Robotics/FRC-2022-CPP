
#include "commands/drivetrain/TurnCommand.h"

TurnCommand::TurnCommand(std::shared_ptr<Drivetrain> drivetrain, double angle,
                         double max_output) {
    // Set everything.
    drivetrain_ = drivetrain;
    angle_ = angle;
    max_output_ = max_output;

    this->AddRequirements(drivetrain_.get());
}

void TurnCommand::Initialize() {
    VOKC_CALL(drivetrain_->ResetTurnPID());
    VOKC_CALL(drivetrain_->ResetEncoders());
    VOKC_CALL(drivetrain_->ResetGyro());
    VOKC_CALL(drivetrain_->SetMaxOutput(max_output_));
}

void TurnCommand::Execute() {
    VOKC_CALL(drivetrain_->TurnToHeading(angle_));
}

bool TurnCommand::IsFinished() {
    bool turn_achieved = false;
    if (drivetrain_->IsAtTurnSetpoint(&turn_achieved)) {
        return turn_achieved;
    }

    // If the check of the setpoint failed, end the command.
    return true;
}
