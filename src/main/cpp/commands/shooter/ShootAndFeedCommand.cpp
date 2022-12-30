
#include "commands/shooter/ShootAndFeedCommand.h"

void ShootAndFeedCommand::Execute() {
    VOKC_CHECK(shooter_ != nullptr);
    VOKC_CALL(shooter_->SetShooter(rpm_));
}
