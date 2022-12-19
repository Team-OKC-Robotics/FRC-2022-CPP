
#include "subsystems/Drivetrain.h"

bool Drivetrain::Init() {
    // Set the open loop ramp rates for the motors.
    // for autonomous start with a fast ramp rate (not too fast otherwise we
    // kind of break the gearboxes) we had problems with stripping gears and
    // whatnot because we put too much force on the gears so this limits that.
    // VEX guy said on Chief Delphi not to put too much acceleration on them
    // otherwise they strip
    OKC_CALL(SetOpenLoopRamp(0.01));

    // Set PID tolerances
    dist_pid_.SetTolerance(2);
    heading_pid_.SetTolerance(7, 1);
    turn_pid_.SetTolerance(7, 2);

    // Get PID gains from parameters
    double dist_p = RobotParams::GetParam("drivetrain.distance_pid.Kp", 0);
    double dist_i = RobotParams::GetParam("drivetrain.distance_pid.Ki", 0);
    double dist_d = RobotParams::GetParam("drivetrain.distance_pid.Kp", 0);

    double heading_p = RobotParams::GetParam("drivetrain.heading_pid.Kp", 0);
    double heading_i = RobotParams::GetParam("drivetrain.heading_pid.Ki", 0);
    double heading_d = RobotParams::GetParam("drivetrain.heading_pid.Kp", 0);

    double turn_p = RobotParams::GetParam("drivetrain.turn_pid.Kp", 0);
    double turn_i = RobotParams::GetParam("drivetrain.turn_pid.Ki", 0);
    double turn_d = RobotParams::GetParam("drivetrain.turn_pid.Kp", 0);

    // Set PID gains.
    dist_pid_.SetPID(dist_p, dist_i, dist_d);
    heading_pid_.SetPID(heading_p, heading_i, heading_d);
    turn_pid_.SetPID(turn_p, turn_i, turn_d);

    // Initialize Shuffleboard from parameters.
    OKC_CALL(InitShuffleboard());

    // Reset everything
    OKC_CALL(ResetEncoders());
    OKC_CALL(ResetGyro());
    OKC_CALL(ResetDistancePID());
    OKC_CALL(ResetHeadingPID());
    OKC_CALL(ResetTurnPID());

    return true;
}

void Drivetrain::Periodic() {
    // Update shuffleboard
    VOKC_CALL(UpdateShuffleboard());
}

void Drivetrain::SimulationPeriodic() {
    // SimulationPeriodic
}

bool Drivetrain::SetSpeedModifier(const double &speed_mod) {
    speed_modifier_ = speed_mod;

    return true;
}

bool Drivetrain::SetOpenLoopRamp(const double &open_loop_ramp) {
    OKC_CHECK(interface_ != nullptr);

    // Set the value
    interface_->drive_config.open_loop_ramp_rate = open_loop_ramp;

    // Notify the I/O system to update the configuration
    interface_->update_config = true;

    return true;
}

bool Drivetrain::CurvatureDrive(const double &speed, const double &turn,
                                const bool turn_in_place) {
    OKC_CHECK(interface_ != nullptr);

    interface_->arcade_power = speed;
    interface_->arcade_turn = turn;
    interface_->drive_mode = DriveMode::CURVATURE;
    interface_->turn_in_place = turn_in_place;

    return true;
}

bool Drivetrain::TankDrive(const double &left_speed,
                           const double &right_speed) {
    OKC_CHECK(interface_ != nullptr);

    interface_->tank_left = left_speed * speed_modifier_;
    interface_->tank_right = right_speed * speed_modifier_;
    interface_->drive_mode = DriveMode::TANK;
    interface_->square_inputs = true;

    return true;
}

bool Drivetrain::ArcadeDrive(const double &speed, const double &turn,
                             bool square_inputs) {
    OKC_CHECK(interface_ != nullptr);

    interface_->arcade_power = speed * speed_modifier_;
    interface_->arcade_turn = turn * speed_modifier_;
    interface_->drive_mode = DriveMode::ARCADE;
    interface_->square_inputs = square_inputs;

    return true;
}

bool Drivetrain::ArcadeDriveAuto(const double &speed, const double &turn,
                                 bool square_inputs) {
    OKC_CHECK(interface_ != nullptr);

    interface_->arcade_power = -1.0 * speed;
    interface_->arcade_turn = turn;
    interface_->square_inputs = square_inputs;
    interface_->drive_mode = DriveMode::ARCADE;

    return true;
}

bool Drivetrain::DriveDistance(const double &distance) {
    // Set the setpoint.
    dist_pid_.SetSetpoint(distance);

    // Get the current encoder average
    double encoder_avg = 0.0;
    OKC_CALL(this->GetEncoderAverage(&encoder_avg));

    // Convert encoder to inches
    double current_dist = 0.0;
    OKC_CALL(this->GetInches(encoder_avg, &current_dist));

    // Get the heading of the robot.
    double heading = 0.0;
    OKC_CALL(this->GetHeading(&heading));

    OKC_CALL(ArcadeDrive(dist_pid_.Calculate(current_dist),
                         heading_pid_.Calculate(heading)));

    return true;
}

bool Drivetrain::DriveOnHeading(const double &speed, const double &distance) {
    // Set the setpoint.
    dist_pid_.SetSetpoint(distance);

    // Compute arcade drive inputs
    double encoder_avg = 0.0;
    OKC_CALL(this->GetEncoderAverage(&encoder_avg));

    double cmd_speed = dist_pid_.Calculate(encoder_avg);
    OKC_CALL(TeamOKC::Clamp(-speed, speed, &cmd_speed));

    double heading = 0.0;
    OKC_CALL(this->GetHeading(&heading));
    double cmd_turn = heading_pid_.Calculate(heading);

    OKC_CALL(ArcadeDriveAuto(cmd_speed, cmd_turn, false));

    return true;
}

bool Drivetrain::TurnToHeading(const double &heading) {
    // Set the turn setpoint
    turn_pid_.SetSetpoint(heading);

    // Compute arcade drive inputs
    double cur_heading = 0.0;
    OKC_CALL(this->GetHeading(&cur_heading));
    double cmd_turn = turn_pid_.Calculate(cur_heading);

    OKC_CALL(ArcadeDrive(0, cmd_turn));

    return true;
}

bool Drivetrain::SetHeading() {
    // From Java code: unclear why this function exists
    // TODO: refactor ported code.

    // Get the current heading and set it to be the setpoint.
    double cur_heading = 0.0;
    OKC_CALL(this->GetHeading(&cur_heading));
    heading_pid_.SetSetpoint(cur_heading);

    return true;
}

bool Drivetrain::GetEncoderAverage(double *avg) {
    OKC_CHECK(avg != nullptr);

    double left_avg = 0.0;
    OKC_CALL(this->GetLeftEncoderAverage(&left_avg));

    double right_avg = 0.0;
    OKC_CALL(this->GetRightEncoderAverage(&right_avg));

    // Compute the average between the left and right sides.
    *avg = (left_avg + right_avg) / 2.0;

    return true;
}

bool Drivetrain::GetLeftEncoderAverage(double *left_avg) {
    OKC_CHECK(left_avg != nullptr);
    OKC_CHECK(interface_ != nullptr);

    // Left encoder 1 is used to represent the whole drivetrain.
    *left_avg = interface_->left_encoder_avg;

    return true;
}

bool Drivetrain::GetRightEncoderAverage(double *right_avg) {
    OKC_CHECK(right_avg != nullptr);
    OKC_CHECK(interface_ != nullptr);

    // Left encoder 1 is used to represent the whole drivetrain.
    *right_avg = interface_->right_encoder_avg;

    return true;
}

bool Drivetrain::ResetEncoders() {
    OKC_CHECK(interface_ != nullptr);

    // Reset the encoders.
    interface_->reset_encoders = true;

    // Update encoder UI
    DrivetrainUI::nt_left_ticks.SetDouble(0);
    DrivetrainUI::nt_right_ticks.SetDouble(0);
    DrivetrainUI::nt_total_ticks.SetDouble(0);

    return true;
}

bool Drivetrain::GetInches(const double &rotations, double *inches) {
    OKC_CHECK(inches != nullptr);

    double gear_ratio = RobotParams::GetParam("drivetrain.gear_ratio", 1);
    double wheel_diameter =
        RobotParams::GetParam("drivetrain.wheel_diameter", 1);

    *inches = rotations * gear_ratio * wheel_diameter * M_PI;

    return true;
}

bool Drivetrain::IsAtDistanceSetpoint(bool *at_setpoint) {
    OKC_CHECK(at_setpoint != nullptr);

    *at_setpoint = dist_pid_.AtSetpoint();

    return true;
}

bool Drivetrain::IsAtHeadingSetpoint(bool *at_setpoint) {
    OKC_CHECK(at_setpoint != nullptr);

    *at_setpoint = heading_pid_.AtSetpoint();

    return true;
}

bool Drivetrain::IsAtTurnSetpoint(bool *at_setpoint) {
    OKC_CHECK(at_setpoint != nullptr);

    *at_setpoint = turn_pid_.AtSetpoint();

    return true;
}

bool Drivetrain::ResetDistancePID() {
    dist_pid_.Reset();
    return true;
}

bool Drivetrain::ResetHeadingPID() {
    heading_pid_.Reset();
    return true;
}

bool Drivetrain::ResetTurnPID() {
    turn_pid_.Reset();
    return true;
}

bool Drivetrain::GetHeading(double *heading) {
    OKC_CHECK(heading != nullptr);
    OKC_CHECK(interface_ != nullptr);

    *heading = interface_->imu_yaw;

    return true;
}

bool Drivetrain::ResetGyro() {
    OKC_CHECK(interface_ != nullptr);

    interface_->reset_gyro = true;

    return true;
}

bool Drivetrain::SetMaxOutput(const double &max_output) {
    OKC_CHECK(interface_ != nullptr);

    interface_->drive_config.max_output = max_output;

    return true;
}

bool Drivetrain::InitShuffleboard() {
    // Get parameters
    double dist_p = RobotParams::GetParam("drivetrain.distance_pid.Kp", 0);
    double dist_i = RobotParams::GetParam("drivetrain.distance_pid.Ki", 0);
    double dist_d = RobotParams::GetParam("drivetrain.distance_pid.Kp", 0);

    double heading_p = RobotParams::GetParam("drivetrain.heading_pid.Kp", 0);
    double heading_i = RobotParams::GetParam("drivetrain.heading_pid.Ki", 0);
    double heading_d = RobotParams::GetParam("drivetrain.heading_pid.Kp", 0);

    double turn_p = RobotParams::GetParam("drivetrain.turn_pid.Kp", 0);
    double turn_i = RobotParams::GetParam("drivetrain.turn_pid.Ki", 0);
    double turn_d = RobotParams::GetParam("drivetrain.turn_pid.Kp", 0);

    // Update dashboard.
    DrivetrainUI::nt_dist_kp.SetDouble(dist_p);
    DrivetrainUI::nt_dist_ki.SetDouble(dist_i);
    DrivetrainUI::nt_dist_kd.SetDouble(dist_d);

    DrivetrainUI::nt_heading_kp.SetDouble(heading_p);
    DrivetrainUI::nt_heading_ki.SetDouble(heading_i);
    DrivetrainUI::nt_heading_kd.SetDouble(heading_d);

    DrivetrainUI::nt_turn_kp.SetDouble(turn_p);
    DrivetrainUI::nt_turn_ki.SetDouble(turn_i);
    DrivetrainUI::nt_turn_kd.SetDouble(turn_d);

    return true;
}

bool Drivetrain::UpdateShuffleboard() {
    // If competition mode isn't set to true, then allow the PID gains to be
    // tuned.
    bool is_competition = RobotParams::GetParam("competition", false);
    if (!is_competition) {
        // Update encoder UI
        double encoder_tmp = 0.0;
        OKC_CALL(GetLeftEncoderAverage(&encoder_tmp));
        DrivetrainUI::nt_left_ticks.SetDouble(encoder_tmp);
        OKC_CALL(GetRightEncoderAverage(&encoder_tmp));
        DrivetrainUI::nt_right_ticks.SetDouble(encoder_tmp);
        OKC_CALL(GetEncoderAverage(&encoder_tmp));
        DrivetrainUI::nt_total_ticks.SetDouble(encoder_tmp);

        // Heading UI
        double heading_tmp = 0.0;
        OKC_CALL(GetHeading(&heading_tmp));
        DrivetrainUI::nt_heading.SetDouble(heading_tmp);

        // Distance UI
        double dist_err = dist_pid_.GetPositionError();
        DrivetrainUI::nt_dist_error.SetDouble(dist_err);

        // Update the PID Gains if write mode is true.
        if (DrivetrainUI::nt_write_mode.GetBoolean(false)) {
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
            dist_p = DrivetrainUI::nt_dist_kp.GetDouble(dist_p);
            dist_i = DrivetrainUI::nt_dist_ki.GetDouble(dist_i);
            dist_d = DrivetrainUI::nt_dist_kd.GetDouble(dist_d);

            heading_p = DrivetrainUI::nt_heading_kp.GetDouble(heading_p);
            heading_i = DrivetrainUI::nt_heading_ki.GetDouble(heading_i);
            heading_d = DrivetrainUI::nt_heading_kd.GetDouble(heading_d);

            turn_p = DrivetrainUI::nt_turn_kp.GetDouble(turn_p);
            turn_i = DrivetrainUI::nt_turn_ki.GetDouble(turn_i);
            turn_d = DrivetrainUI::nt_turn_kd.GetDouble(turn_d);

            // Distance PID
            dist_pid_.SetPID(dist_p, dist_i, dist_d);
            heading_pid_.SetPID(heading_p, heading_i, heading_d);
            turn_pid_.SetPID(turn_p, turn_i, turn_d);
        }
    }

    // Resetting the Gyro needs to always be available.
    if (DrivetrainUI::nt_reset_gyro.GetBoolean(false)) {
        interface_->reset_gyro = true;
        DrivetrainUI::nt_reset_gyro.SetBoolean(false);
    }

    return true;
}