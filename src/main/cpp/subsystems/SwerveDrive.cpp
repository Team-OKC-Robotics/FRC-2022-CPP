
#include "subsystems/SwerveDrive.h"


bool SwerveDrive::Init() {
    // Initialize Shuffleboard from parameters.
    OKC_CALL(InitShuffleboard());

    double x_disp = RobotParams::GetParam("swerve.x_disp", 0.3); // in meters
    double y_disp = RobotParams::GetParam("swerve.y_disp", 0.3); // in meters

    // PID Controller stuff
    // drive PID gains
    double drive_kP = RobotParams::GetParam("swerve.drive_pid.kP", 0.0);
    double drive_kI = RobotParams::GetParam("swerve.drive_pid.kI", 0.001);
    double drive_kD = RobotParams::GetParam("swerve.drive_pid.kD", 0.0001);

    // steer PID gains
    double steer_kP = RobotParams::GetParam("swerve.steer_pid.kP", 0.0);
    double steer_kI = RobotParams::GetParam("swerve.steer_pid.kI", 0.001);
    double steer_kD = RobotParams::GetParam("swerve.steer_pid.kD", 0.0001);

    left_front_drive_pid = frc::PIDController(drive_kP, drive_kI, drive_kD);
    left_back_drive_pid = frc::PIDController(drive_kP, drive_kI, drive_kD);
    right_front_drive_pid = frc::PIDController(drive_kP, drive_kI, drive_kD);
    right_back_drive_pid = frc::PIDController(drive_kP, drive_kI, drive_kD);

    left_front_steer_pid = frc::PIDController(steer_kP, steer_kI, steer_kD);
    left_back_steer_pid = frc::PIDController(steer_kP, steer_kI, steer_kD);
    right_front_steer_pid = frc::PIDController(steer_kP, steer_kI, steer_kD);
    right_back_steer_pid = frc::PIDController(steer_kP, steer_kI, steer_kD);



    // !! IMPORTANT ASSUMPTION/PRACTICE/WHATEVER !!
    // the order of swerve stuff should always be in:
    // left front, left back, right front, right back
    // for the sake of consistency and also if you want the swerve drive to actually work
    // because that should be how the argumetns are passed in, and however they're passed in,
    // that's how they gonna get passed out

    // define SwerveKinematics object
    swerve_kinematics = frc::SwerveDriveKinematics<4>(frc::Translation2d(units::meter_t(x_disp), units::meter_t(y_disp)), frc::Translation2d(units::meter_t(-x_disp), units::meter_t(y_disp)), frc::Translation2d(units::meter_t(x_disp), units::meter_t(-y_disp)), frc::Translation2d(units::meter_t(-x_disp), units::meter_t(-y_disp)));

    // define SwerveOdometry object
    swerve_odometry = frc::SwerveDriveOdometry(swerve_kinematics, frc::Rotation2d(), modules, frc::Pose2d());

    // define SwerveModuleStates objects
    // everything starts at a default of 0 m/s and 0 degrees
    left_front_module = frc::SwerveModuleState(units::meters_per_second_t(0.0), frc::Rotation2d());
    left_back_module = frc::SwerveModuleState(units::meters_per_second_t(0.0), frc::Rotation2d());
    right_front_module = frc::SwerveModuleState(units::meters_per_second_t(0.0), frc::Rotation2d());
    right_back_module = frc::SwerveModuleState(units::meters_per_second_t(0.0), frc::Rotation2d());

    // define SwerveModulePosition objects
    // everything starts at a default of 0m and 0 degrees
    left_front_pos = frc::SwerveModulePosition(units::meter_t(0.0), frc::Rotation2d());
    left_back_pos = frc::SwerveModulePosition(units::meter_t(0.0), frc::Rotation2d());
    right_front_pos = frc::SwerveModulePosition(units::meter_t(0.0), frc::Rotation2d());
    right_back_pos = frc::SwerveModulePosition(units::meter_t(0.0), frc::Rotation2d());

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
    // get outputs from the kinematics object based on joystick inputs
    outputs = swerve_kinematics.ToSwerveModuleStates(frc::ChassisSpeeds(units::meters_per_second_t(drive), units::meters_per_second_t(strafe), units::radians_per_second_t(turn)));
    
    for(int i = 0; i < outputs.size(); i++) {
        // optimize the angle of the wheel so we never turn more than 90 degrees or something like that
        // basically we avoid unnecessary turning because positive negative angle stuff
        outputs.at(i) = frc::SwerveModuleState::Optimize(modules.at(i).angle.Radians(), outputs.at(i).angle.Radians());
    }

    // set all the outputs in the interface
    interface_->left_front_drive_motor_output = outputs.at(0).speed;
    interface_->left_back_drive_motor_output = outputs.at(0).angle;

    interface_->right_front_drive_motor_output = outputs.at(1).speed;
    interface_->right_back_drive_motor_output = outputs.at(1).angle;

    interface_->left_front_steer_motor_output = outputs.at(2).speed;
    interface_->left_back_steer_motor_output = outputs.at(2).angle;

    interface_->right_front_steer_motor_output = outputs.at(3).speed;
    interface_->right_back_steer_motor_output = outputs.at(3).angle;

    // return true
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