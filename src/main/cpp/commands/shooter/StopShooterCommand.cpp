
#include "commands/shooter/StopShooterCommand.h"

void StopShooterCommand::Execute() {
    VOKC_CHECK(shooter_ != nullptr);

    VOKC_CALL(shooter_->StopShooter());
    VOKC_CALL(shooter_->ResetPower());
}

bool StopShooterCommand::IsFinished() {
    // Always end.
    return true;
}
