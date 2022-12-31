
#include "subsystems/SwerveDrive.h"
#include "units/length.h"
#include "frc/geometry/Rotation2d.h"

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

    // get the heading
    double heading = 0;
    VOKC_CALL(this->GetHeading(&heading));

    // update modules
    //TODO figure out the conversion between steer motor encoder reading and actual reading in degees
    modules[0] = frc::SwerveModulePosition(units::meter_t(this->interface_->left_front_drive_motor_enc), frc::Rotation2d(units::degree_t(this->interface_->left_front_steer_motor_enc)));
    modules[1] = frc::SwerveModulePosition(units::meter_t(this->interface_->left_back_drive_motor_enc), frc::Rotation2d(units::degree_t(this->interface_->left_back_steer_motor_enc)));
    modules[2] = frc::SwerveModulePosition(units::meter_t(this->interface_->right_front_drive_motor_enc), frc::Rotation2d(units::degree_t(this->interface_->right_front_steer_motor_enc)));
    modules[3] = frc::SwerveModulePosition(units::meter_t(this->interface_->right_back_drive_motor_enc), frc::Rotation2d(units::degree_t(this->interface_->right_back_steer_motor_enc)));


    // update odometry
    swerve_odometry.Update(frc::Rotation2d(units::degree_t(heading)), modules); // negate heading because odometry expects counterclockwise to be positive, but the NavX is not
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
    //TODO

    // Update dashboard.
    //TODO

    return true;
}

bool SwerveDrive::UpdateShuffleboard() {
    // If competition mode isn't set to true, then allow the PID gains to be
    // tuned.
    bool is_competition = RobotParams::GetParam("competition", false);
    if (!is_competition) {
        /*// Update encoder UI
        //TODO
   
        // Heading UI
        double heading_tmp = 0.0;
        OKC_CALL(GetHeading(&heading_tmp));
        SwerveDriveUI::nt_heading.SetDouble(heading_tmp);

        // Distance UI
        //TODO
      
        // Update the PID Gains if write mode is true.
        //TODO

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
    }*/
    }

    return true;
}