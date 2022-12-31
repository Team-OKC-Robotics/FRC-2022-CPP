
#include "subsystems/SwerveDrive.h"

bool SwerveDrive::Init() {
    // Initialize Shuffleboard from parameters.
    OKC_CALL(InitShuffleboard());

    double x_disp = RobotParams::GetParam("swerve.x_disp", 0.3); // in meters
    double y_disp = RobotParams::GetParam("swerve.y_disp", 0.3); // in meters

    // !! IMPORTANT ASSUMPTION/PRACTICE/WHATEVER !!
    // the order of swerve stuff should always be in:
    // left front, left back, right front, right back
    // for the sake of consistency and also if you want the swerve drive to actually work
    // because that should be how the argumetns are passed in, and however they're passed in,
    // that's how they gonna get passed out

    //TODO define displacements (the Translation2d stuff)

    //TODO define SwerveKinematics object

    //TODO define SwerveOdometry object

    //TODO define SwerveModuleStates objects



    // Reset everything
    OKC_CALL(ResetDriveEncoders());
    OKC_CALL(ResetSteerEncoders());
    OKC_CALL(ResetGyro());
    return true;
}

void SwerveDrive::Periodic() {
    // Update shuffleboard
    VOKC_CALL(UpdateShuffleboard());

    // update odometry
    swerve_odometry.update(-this->GetHeading()); // negate heading because odometry expects counterclockwise to be positive, but the NavX is not
}

void SwerveDrive::SimulationPeriodic() {
    // SimulationPeriodic
}

bool SwerveDrive::SetSpeedModifierDrive(const double &speed_mod) {
    speed_modifier_drive = speed_mod;

    return true;
}

bool SwerveDrive::SetSpeedModifierSteer(const double &speed_mod) {
    speed_modifier_steer = speed_mod;
    
    return true;
}

bool SwerveDrive::SetOpenLoopRampDrive(const double &open_loop_ramp) {
    OKC_CHECK(interface_ != nullptr);

    interface_->drive_config.open_loop_ramp_rate_drive = open_loop_ramp;
    return true;
}

bool SwerveDrive::SetOpenLoopRampSteer(const double &open_loop_ramp) {
    OKC_CHECK(interface_ != nullptr);

    interface_->drive_config.open_loop_ramp_rate_steer = open_loop_ramp;
    return true;
}

bool SwerveDrive::SetMaxOutputDrive(const double &max_output) {
    OKC_CHECK(interface_ != nullptr);

    interface_->drive_config.max_output_drive = max_output;
    return true;
}

bool SwerveDrive::SetMaxOutputSteer(const double &max_output) {
    OKC_CHECK(interface_ != nullptr);

    interface_->drive_config.max_output_steer = max_output;
    return true;
}

bool SwerveDrive::TeleOpDrive(const double &drive, const double &strafe, const double &turn) {
    //TODO
    return true;
}


bool SwerveDrive::TranslateAuto(const double &x, const double &y, const double &rot) {
    //TODO
    return true;
}

bool SwerveDrive::TurnToHeading(const double &heading) {
    //TODO
    return true;
}

bool SwerveDrive::GetHeading(double *heading) {
    OKC_CHECK(heading != nullptr);
    OKC_CHECK(interface_ != nullptr);

    heading = &interface_->imu_yaw;

    return true;
}

bool SwerveDrive::ResetDriveEncoders() {
    OKC_CHECK(interface_ != nullptr);

    interface_->reset_drive_encoders = true;
    return true;
}

bool SwerveDrive::ResetSteerEncoders() {
    OKC_CHECK(interface_ != nullptr);

    interface_->reset_steer_encoders = true;
    return true;
}


bool SwerveDrive::ResetGyro() {
    OKC_CHECK(interface_ != nullptr);

    interface_->reset_gyro = true;
    return true;
}


bool SwerveDrive::InitShuffleboard() {
    // Get parameters
    // Update dashboard.
    return true;
}

bool SwerveDrive::UpdateShuffleboard() {
    // If competition mode isn't set to true, then allow the PID gains to be
    // tuned.
    bool is_competition = RobotParams::GetParam("competition", false);
    if (!is_competition) {
        // Update encoder UI
        double encoder_tmp = 0.0;
        OKC_CALL(GetLeftEncoderAverage(&encoder_tmp));
        SwerveDriveUI::nt_left_ticks.SetDouble(encoder_tmp);
        OKC_CALL(GetRightEncoderAverage(&encoder_tmp));
        SwerveDriveUI::nt_right_ticks.SetDouble(encoder_tmp);
        OKC_CALL(GetEncoderAverage(&encoder_tmp));
        SwerveDriveUI::nt_total_ticks.SetDouble(encoder_tmp);

        // Heading UI
        double heading_tmp = 0.0;
        OKC_CALL(GetHeading(&heading_tmp));
        SwerveDriveUI::nt_heading.SetDouble(heading_tmp);

        // Distance UI
        double dist_err = dist_pid_.GetPositionError();
        SwerveDriveUI::nt_dist_error.SetDouble(dist_err);

        // Update the PID Gains if write mode is true.
        if (SwerveDriveUI::nt_write_mode.GetBoolean(false)) {
            // Get the current PID parameter values
            double dist_p =
                RobotParams::GetParam("drivetrain.distance_pid.Kp", 0);
            double dist_i =
                RobotParams::GetParam("drivetrain.distance_pid.Ki", 0);
            double dist_d =
                RobotParams::GetParam("drivetrain.distance_pid.Kp", 0);

            double heading_p =
                RobotParams::GetParam("drivetrain.heading_pid.Kp", 0);
            double heading_i =
                RobotParams::GetParam("drivetrain.heading_pid.Ki", 0);
            double heading_d =
                RobotParams::GetParam("drivetrain.heading_pid.Kp", 0);

            double turn_p = RobotParams::GetParam("drivetrain.turn_pid.Kp", 0);
            double turn_i = RobotParams::GetParam("drivetrain.turn_pid.Ki", 0);
            double turn_d = RobotParams::GetParam("drivetrain.turn_pid.Kp", 0);

            // Get the values from shuffleboard.
            dist_p = SwerveDriveUI::nt_dist_kp.GetDouble(dist_p);
            dist_i = SwerveDriveUI::nt_dist_ki.GetDouble(dist_i);
            dist_d = SwerveDriveUI::nt_dist_kd.GetDouble(dist_d);

            heading_p = SwerveDriveUI::nt_heading_kp.GetDouble(heading_p);
            heading_i = SwerveDriveUI::nt_heading_ki.GetDouble(heading_i);
            heading_d = SwerveDriveUI::nt_heading_kd.GetDouble(heading_d);

            turn_p = SwerveDriveUI::nt_turn_kp.GetDouble(turn_p);
            turn_i = SwerveDriveUI::nt_turn_ki.GetDouble(turn_i);
            turn_d = SwerveDriveUI::nt_turn_kd.GetDouble(turn_d);

            // Distance PID
            dist_pid_.SetPID(dist_p, dist_i, dist_d);
            heading_pid_.SetPID(heading_p, heading_i, heading_d);
            turn_pid_.SetPID(turn_p, turn_i, turn_d);
        }

        // Allow saving parameters in non-competition modes
        if (SwerveDriveUI::nt_save.GetBoolean(true)) {
            // Save the parameters.
            OKC_CALL(RobotParams::SaveParameters(RobotParams::param_file));
            SwerveDriveUI::nt_save.SetBoolean(false);
        }
    }

    // Resetting the Gyro needs to always be available.
    if (SwerveDriveUI::nt_reset_gyro.GetBoolean(false)) {
        interface_->reset_gyro = true;
        SwerveDriveUI::nt_reset_gyro.SetBoolean(false);
    }

    return true;
}