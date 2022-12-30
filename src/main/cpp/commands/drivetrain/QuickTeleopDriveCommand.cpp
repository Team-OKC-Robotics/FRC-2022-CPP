
#include "commands/drivetrain/QuickTeleopDriveCommand.h"

QuickTeleopDriveCommand::QuickTeleopDriveCommand(
    std::shared_ptr<Drivetrain> drivetrain,
    std::shared_ptr<frc::Joystick> gamepad) {
    // Set everything.
    drivetrain_ = drivetrain;
    gamepad_ = gamepad;

    if (drivetrain_ != nullptr) {
        this->AddRequirements(drivetrain_.get());
    }
}

void QuickTeleopDriveCommand::Initialize() {
    VOKC_CHECK(drivetrain_ != nullptr);

    VOKC_CALL(drivetrain_->SetOpenLoopRamp(0.1));
    VOKC_CALL(drivetrain_->SetSpeedModifier(1.0));
}

void QuickTeleopDriveCommand::Execute() {
    VOKC_CHECK(drivetrain_ != nullptr);
    VOKC_CHECK(gamepad_ != nullptr);

    double throttle = -1.0 * pow(gamepad_->GetRawAxis(1), 3);
    double turn_power = pow(gamepad_->GetRawAxis(4), 3);
    VOKC_CALL(drivetrain_->ArcadeDrive(throttle, turn_power, false));
}

bool QuickTeleopDriveCommand::IsFinished() {
    return false;
}
