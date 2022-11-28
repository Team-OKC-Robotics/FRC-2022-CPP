
#include "io/DrivetrainIO.h"

void DrivetrainIO::Periodic() {
    // Process all the inputs and outputs to/from high level software.
    VOKC_CALL(ProcessIO());
}

void DrivetrainIO::SimulationPeriodic() {
    // SimulationPeriodic
}

bool DrivetrainIO::ProcessIO() {
    OKC_CHECK(sw_interface_ != nullptr);
    OKC_CHECK(hw_interface_ != nullptr);

    // Get the hardware sensor values.
    // navX IMU:
    sw_interface_->imu_yaw = hw_interface_->ahrs->GetAngle();

    // Encoders
    sw_interface_->left_encoder_avg =
        hw_interface_->left_motor_1->GetEncoder().GetPosition();
    sw_interface_->right_encoder_avg =
        hw_interface_->right_motor_1->GetEncoder().GetPosition();

    // Set the software outputs
    // If the drivetrain configuration needs to be updated, update it.
    if (sw_interface_->update_config) {
        OKC_CALL(UpdateDriveConfig(sw_interface_->drive_config));

        // Lower the update flag
        sw_interface_->update_config = false;
    }

    // If the encoders should be reset, reset them
    if (sw_interface_->reset_encoders) {
        OKC_CALL(ResetEncoders());

        // Lower the encoder reset flag
        sw_interface_->reset_encoders = false;
    }

    // If the navX should be reset, reset it.
    if (sw_interface_->reset_gyro) {
        hw_interface_->ahrs->Reset();
    }

    // Set the drive outputs based on the drive mode.
    switch (sw_interface_->drive_mode) {
    case DriveMode::ARCADE:
        hw_interface_->diff_drive->ArcadeDrive(sw_interface_->arcade_power,
                                               sw_interface_->arcade_turn,
                                               sw_interface_->square_inputs);
        break;
    case DriveMode::TANK:
        hw_interface_->diff_drive->TankDrive(sw_interface_->tank_left,
                                             sw_interface_->tank_right,
                                             sw_interface_->square_inputs);
        break;
    case DriveMode::CURVATURE:
        hw_interface_->diff_drive->CurvatureDrive(sw_interface_->arcade_power,
                                                  sw_interface_->arcade_turn,
                                                  sw_interface_->turn_in_place);
        break;
    default:
        // We should not reach this case, return false to indicate an error
        // occured.
        return false;
    }

    return true;
}

bool DrivetrainIO::UpdateDriveConfig(DrivetrainConfig &config) {
    OKC_CHECK(hw_interface_ != nullptr);

    // Get the configuration
    double open_loop_ramp = config.open_loop_ramp_rate;
    double max_output = config.max_output;

    // Apply the configuration
    // Open Loop Ramp Rate
    hw_interface_->left_motor_1->SetOpenLoopRampRate(open_loop_ramp);
    hw_interface_->left_motor_2->SetOpenLoopRampRate(open_loop_ramp);
    hw_interface_->left_motor_3->SetOpenLoopRampRate(open_loop_ramp);
    hw_interface_->right_motor_1->SetOpenLoopRampRate(open_loop_ramp);
    hw_interface_->right_motor_2->SetOpenLoopRampRate(open_loop_ramp);
    hw_interface_->right_motor_3->SetOpenLoopRampRate(open_loop_ramp);

    // Drivetrain Max Output
    hw_interface_->diff_drive->SetMaxOutput(max_output);

    return true;
}

bool DrivetrainIO::ResetEncoders() {
    OKC_CHECK(hw_interface_ != nullptr);

    hw_interface_->left_motor_1->GetEncoder().SetPosition(0.0);
    hw_interface_->left_motor_2->GetEncoder().SetPosition(0.0);
    hw_interface_->left_motor_3->GetEncoder().SetPosition(0.0);
    hw_interface_->right_motor_1->GetEncoder().SetPosition(0.0);
    hw_interface_->right_motor_2->GetEncoder().SetPosition(0.0);
    hw_interface_->right_motor_3->GetEncoder().SetPosition(0.0);

    return true;
}
