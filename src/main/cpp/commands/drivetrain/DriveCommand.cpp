
#include "commands/drivetrain/DriveCommand.h"

DriveCommand::DriveCommand(std::shared_ptr<Drivetrain> drivetrain,
                           double distance, double max_output) {
    // Set everything.
    drivetrain_ = drivetrain;
    distance_ = distance;
    max_output_ = max_output;

    this->AddRequirements(drivetrain_.get());
}

void DriveCommand::Initialize() {
    VOKC_CALL(drivetrain_->ResetEncoders());
    VOKC_CALL(drivetrain_->ResetDistancePID());
    VOKC_CALL(drivetrain_->ResetHeadingPID());
    VOKC_CALL(drivetrain_->SetHeading());
    VOKC_CALL(drivetrain_->SetMaxOutput(max_output_));
}

void DriveCommand::Execute() {
    VOKC_CALL(drivetrain_->DriveDistance(distance_));
}

void DriveCommand::End(bool executed) {
    VOKC_CALL(drivetrain_->SetMaxOutput(1.0));
}

bool DriveCommand::IsFinished() {
    bool distance_achieved = false;
    bool heading_achieved = false;

    if ((drivetrain_->IsAtDistanceSetpoint(&distance_achieved)) &&
        (drivetrain_->IsAtHeadingSetpoint(&heading_achieved))) {
        return (distance_achieved && heading_achieved);
    }

    // If either check fails, end this command.
    return true;
}
