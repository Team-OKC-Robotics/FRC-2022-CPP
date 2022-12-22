
#include "commands/drivetrain/DriveSetSpeedCommand.h"

DriveSetSpeedCommand::DriveSetSpeedCommand(
    std::shared_ptr<Drivetrain> drivetrain, double distance, double speed) {
    // Set everything.
    drivetrain_ = drivetrain;
    distance_ = distance;
    speed_ = speed;

    this->AddRequirements(drivetrain_.get());
}

void DriveSetSpeedCommand::Initialize() {
    VOKC_CALL(drivetrain_->ResetEncoders());
    VOKC_CALL(drivetrain_->ResetDistancePID());
    VOKC_CALL(drivetrain_->ResetHeadingPID());
    VOKC_CALL(drivetrain_->SetHeading());
    VOKC_CALL(drivetrain_->ResetGyro());
}

void DriveSetSpeedCommand::Execute() {
    VOKC_CALL(drivetrain_->DriveOnHeading(speed_, distance_));
}

bool DriveSetSpeedCommand::IsFinished() {
    bool distance_achieved = false;

    if (drivetrain_->IsAtDistanceSetpoint(&distance_achieved)) {
        return distance_achieved;
    }

    // If the check fails, end the command.
    return true;
}
