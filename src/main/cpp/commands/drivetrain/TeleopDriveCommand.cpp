
#include "commands/drivetrain/TeleopDriveCommand.h"

TeleopDriveCommand::TeleopDriveCommand(std::shared_ptr<Drivetrain> drivetrain,
                                       std::shared_ptr<frc::Joystick> gamepad) {
    // Set everything.
    drivetrain_ = drivetrain;
    gamepad_ = gamepad;

    if (drivetrain_ != nullptr) {
        this->AddRequirements(drivetrain_.get());
    }
}

void TeleopDriveCommand::Initialize() {
    VOKC_CHECK(drivetrain_ != nullptr);

    VOKC_CALL(drivetrain_->SetOpenLoopRamp(0.5));
    VOKC_CALL(drivetrain_->SetSpeedModifier(0.75));
}

void TeleopDriveCommand::Execute() {
    VOKC_CHECK(drivetrain_ != nullptr);
    VOKC_CHECK(gamepad_ != nullptr);

    double throttle = -1.0 * pow(gamepad_->GetRawAxis(1), 3);
    double turn_power = pow(gamepad_->GetRawAxis(4), 3);
    VOKC_CALL(drivetrain_->ArcadeDrive(throttle, turn_power, false));
}

bool TeleopDriveCommand::IsFinished() {
    return false;
}
