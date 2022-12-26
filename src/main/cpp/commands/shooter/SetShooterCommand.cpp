
#include "commands/shooter/SetShooterCommand.h"

SetShooterCommand::SetShooterCommand(std::shared_ptr<Shooter> shooter,
                                     double rpm) {
    // Set everything.
    shooter_ = shooter;
    rpm_ = rpm;

    // Add the shooter as a requirement
    AddRequirements(shooter_.get());
}

void SetShooterCommand::Execute() {
    VOKC_CHECK(shooter_ != nullptr);
    VOKC_CALL(shooter_->SetShooter(rpm_));
}

bool SetShooterCommand::IsFinished() {
    VOKC_CHECK(shooter_ != nullptr);

    bool at_setpoint = true;
    if (shooter_->IsAtShooterSetpoint(&at_setpoint)) {
        // End this command if we are at the setpoint.
        return at_setpoint;
    }

    // End this command if there's an error.
    return true;
}
