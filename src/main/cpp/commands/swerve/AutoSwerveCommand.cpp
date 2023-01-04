
#include "commands/swerve/AutoSwerveCommand.h"

AutoSwerveCommand::AutoSwerveCommand(std::shared_ptr<SwerveDrive> swerve, frc::Translation2d f_pos, frc::Rotation2d f_angle) {
    swerve_ = swerve;
    end_pos = f_pos;
    final_angle = f_angle;

    if (swerve_ != nullptr) {
        this->AddRequirements(swerve_.get());
    }
}

void AutoSwerveCommand::Initialize() {
    VOKC_CHECK(swerve_ != nullptr);

    VOKC_CALL(swerve_->ResetDriveEncoders());
    VOKC_CALL(swerve_->ResetSteerEncoders());
    VOKC_CALL(swerve_->ResetPIDs());
}

void AutoSwerveCommand::Execute() {
    VOKC_CHECK(swerve_ != nullptr);

    VOKC_CALL(swerve_->TranslateAuto(end_pos, final_angle));
}

void AutoSwerveCommand::End(bool executed) {
    return;
}

bool AutoSwerveCommand::IsFinished() {
    // if the swerve subsystem doesn't exist
    if (swerve_ == nullptr) {
        return true; // then don't try to do anything on it because it will fail
    }

    // if the swerve drive is at its setpoint
    if (swerve_->AtSetpoint()) {
        return true; // end the command
    }

    // otherwise, continue
    return false;
}
