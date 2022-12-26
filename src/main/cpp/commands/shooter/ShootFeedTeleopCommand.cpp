
#include "commands/shooter/ShootFeedTeleopCommand.h"

void ShootFeedTeleopCommand::Execute() {
    VOKC_CALL(shooter_->SetShooter(rpm_));
    VOKC_CALL(shooter_->Feed(power_));
}

void ShootFeedTeleopCommand::End(bool executed) {
    VOKC_CALL(shooter_->Feed(0.0));
    VOKC_CALL(shooter_->StopShooter());
}

bool ShootFeedTeleopCommand::IsFinished() {
    return false;
}
