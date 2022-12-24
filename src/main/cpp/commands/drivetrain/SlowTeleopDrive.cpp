
#include "commands/drivetrain/SlowTeleopDrive.h"

SlowTeleopDrive::SlowTeleopDrive(std::shared_ptr<Drivetrain> drivetrain,
                                 std::shared_ptr<frc::Joystick> gamepad) {
    // Set everything.
    drivetrain_ = drivetrain;
    gamepad_ = gamepad;

    this->AddRequirements(drivetrain_.get());
}

void SlowTeleopDrive::Execute() {
    double throttle = -1.0 * gamepad_->GetRawAxis(1) / 3.0;
    double turn_power = gamepad_->GetRawAxis(4) / 3.0;
    VOKC_CALL(drivetrain_->ArcadeDrive(throttle, turn_power, false));
}

bool SlowTeleopDrive::IsFinished() {
    return false;
}
