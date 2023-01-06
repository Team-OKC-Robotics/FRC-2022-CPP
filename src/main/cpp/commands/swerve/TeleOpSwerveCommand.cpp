
#include "commands/swerve/TeleOpSwerveCommand.h"

TeleOpSwerveCommand::TeleOpSwerveCommand(std::shared_ptr<SwerveDrive> swerve, std::shared_ptr<frc::Joystick> gamepad) {
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

    double drive_power = -this->gamepad_->GetRawAxis(1);
    double strafe_power = this->gamepad_->GetRawAxis(2);
    double turn_power = this->gamepad_->GetRawAxis(3);

    VOKC_CALL(swerve_->TeleOpDrive(drive_power, strafe_power, turn_power));
}

bool TeleOpSwerveCommand::IsFinished() {
    // this command should never end, unless it is interrupted by another
    return false;
}
