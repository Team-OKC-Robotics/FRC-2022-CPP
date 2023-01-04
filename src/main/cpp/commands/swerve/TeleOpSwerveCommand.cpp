
#include "commands/swerve/TeleOpSwerveCommand.h"

TeleOpSwerveCommand::TeleOpSwerveCommand(std::shared_ptr<SwerveDrive> swerve, std::shared_ptr<frc::Joystick> gamepad) {
    // Set everything.
    swerve_ = swerve;
    gamepad_ = gamepad;

    if (swerve_ != nullptr) {
        this->AddRequirements(swerve_.get());
    }
}

void TeleOpSwerveCommand::Initialize() {
    VOKC_CHECK(swerve_ != nullptr);
}

void TeleOpSwerveCommand::Execute() {
    VOKC_CHECK(swerve_ != nullptr);
    VOKC_CHECK(gamepad_ != nullptr);

    double throttle = -1.0 * pow(gamepad_->GetRawAxis(1), 3);
    double turn_power = pow(gamepad_->GetRawAxis(4), 3);

    // TODO actually get correct joystick inputs

    VOKC_CALL(swerve_->TeleOpDrive(throttle, turn_power));
}

bool TeleOpSwerveCommand::IsFinished() {
    // this command should never end, unless it is interrupted by another
    return false;
}
