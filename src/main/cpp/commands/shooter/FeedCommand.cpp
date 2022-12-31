
#include "commands/shooter/FeedCommand.h"

FeedCommand::FeedCommand(std::shared_ptr<Shooter> shooter, double power) {
    // Set everything.
    shooter_ = shooter;
    power_ = power;

    // Note: Don't require the shooter since it needs to be used by the
    // ShootCommand while Feed is running.
}

void FeedCommand::Execute() {
    VOKC_CHECK(shooter_ != nullptr);
    VOKC_CALL(shooter_->Feed(power_));
}

bool FeedCommand::IsFinished() {
    // Always end this command.
    return true;
}
