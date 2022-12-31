
#include "commands/shooter/ShooterPresetCommand.h"

void ShooterPresetCommand::Execute() {
    // Get the gamepad POV
    VOKC_CHECK(shooter_ != nullptr);
    VOKC_CHECK(gamepad_ != nullptr);

    ShooterPreset preset = ShooterPreset::NONE;

    switch (gamepad_->GetPOV()) {
    case 0:
        preset = ShooterPreset::AGAINST_HUB;
        break;
    case 180:
        preset = ShooterPreset::NORMAL_SHOT;
        break;
    default:
        break;
    }

    if (preset != ShooterPreset::NONE) {
        VOKC_CALL(shooter_->SetShooterPreset(preset));
    }
}

void ShooterPresetCommand::End(bool executed) {
    // Do nothing for some reason.
}

bool ShooterPresetCommand::IsFinished() {
    // Run until canceled by the StopShooterCommand.
    return false;
}
