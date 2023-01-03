
#include "SwerveModule.h"

bool SwerveModule::Init() {
    // PID Controller stuff
    // drive PID gains
    double drive_kP = RobotParams::GetParam("swerve.drive_pid.kP", 0.0);
    double drive_kI = RobotParams::GetParam("swerve.drive_pid.kI", 0.001);
    double drive_kD = RobotParams::GetParam("swerve.drive_pid.kD", 0.0001);

    // steer PID gains
    double steer_kP = RobotParams::GetParam("swerve.steer_pid.kP", 0.0);
    double steer_kI = RobotParams::GetParam("swerve.steer_pid.kI", 0.001);
    double steer_kD = RobotParams::GetParam("swerve.steer_pid.kD", 0.0001);

    // create our PIDs
    drive_pid = frc::PIDController(drive_kP, drive_kI, drive_kD);
    steer_pid = frc::PIDController(steer_kP, steer_kI, steer_kD);


    // create a default swerve module state with no speed or angle
    state = frc::SwerveModuleState(units::meters_per_second_t(0.0), frc::Rotation2d());

    // create a default swerve module position with no distance traveled or angle    
    pos = frc::SwerveModulePosition(units::meter_t(0.0), frc::Rotation2d());


    // physical location on the robot
    double x_disp = RobotParams::GetParam("swerve.x_disp", 0.3); // in meters
    double y_disp = RobotParams::GetParam("swerve.y_disp", 0.3); // in meters

    // based on where it needs to be
    switch(loc) {
        case Location::LEFT_FRONT:
            // we only have this to pass it into the kinematics object, because WPILib needs it
            // positive x is to the front of the robot, positive y is to the left of the robot
            // this should match the code in the docs pretty well, I think
            trans = frc::Translation2d(units::meter_t(x_disp), units::meter_t(y_disp));
            break;
        case Location::LEFT_BACK:
            trans = frc::Translation2d(units::meter_t(-x_disp), units::meter_t(y_disp));
            break;
        case Location::RIGHT_FRONT:
            trans = frc::Translation2d(frc::Translation2d(units::meter_t(x_disp), units::meter_t(-y_disp)));
            break;
        case Location::RIGHT_BACK:
            trans = frc::Translation2d(frc::Translation2d(units::meter_t(-x_disp), units::meter_t(-y_disp)));
            break;
        default:
            // we shouldn't have reached here, so throw an error
            return false;
    }

    // reset subsystem to initial state
    this->Reset();

    // Init passed succesffully, return true
    return true;
}

bool SwerveModule::Reset() {
    // reset PIDs
    this->drive_pid.Reset();
    this->steer_pid.Reset();

    //TODO reset sensor readings
    //TODO reset WPILib kinematics stuff
    return true;
}

bool SwerveModule::GetLocationOnRobot(frc::Translation2d *trans) {
    // OKC_CHECK(this->trans != nullptr);

    *trans = this->trans;

    return true;
}

bool SwerveModule::GetSwerveModulePosition(frc::SwerveModulePosition *pos) {
    // OKC_CHECK(this->pos != nullptr);

    *pos = this->pos;

    return true;
}

bool SwerveModule::GetSwerveModuleState(frc::SwerveModuleState *state) {
    // OKC_CHECK(this->state != nullptr);

    *state = this->state;
    
    return true;
}

bool SwerveModule::SetDesiredState(frc::SwerveModuleState state) {
    // OKC_CHECK(this->state != nullptr);

    this->state = state;

    return true;
}

bool SwerveModule::GetDriveOutput(double *output) {
    //TODO check for null pointers

    // set setpoint
    this->drive_pid.SetSetpoint(this->state.speed.value());

    // calculate output
    *output = this->drive_pid.Calculate(this->drive_enc_vel);

    return true;
}

bool SwerveModule::GetSteerOutput(double *output) {
    // TODO check for null pointers

    // optimize angle
    this->state = frc::SwerveModuleState::Optimize(this->state, frc::Rotation2d(units::degree_t(this->pos.angle.Degrees())));

    // set setpoint
    this->steer_pid.SetSetpoint(this->state.angle.Degrees().value());

    *output = this->steer_pid.Calculate(this->steer_enc_vel);

    return true;
}

bool SwerveModule::Update(double drive_enc, double steer_enc, double drive_vel, double steer_vel) {
    // update the SwerveModulePosition with the given sensor readings
    
    // 6.75:1 L2 gear ratio
    // wheel is 4 inch diamete wheel
    drive_enc = drive_enc / 6.75 * 3.14159265358979323846264338327950288419716939937510582097494459230781 * 4;

    // steering gear ratio of 12.8:1
    steer_enc = steer_enc / 12.8;

    this->pos = frc::SwerveModulePosition(units::meter_t(drive_enc), frc::Rotation2d(units::degree_t(steer_enc)));

    // velocity readings
    this->drive_enc_vel = drive_vel;
    this->steer_enc_vel = steer_vel;

    return true;
}