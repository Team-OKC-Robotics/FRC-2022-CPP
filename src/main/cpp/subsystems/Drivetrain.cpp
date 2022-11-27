
#include "subsystems/Drivetrain.h"

bool Drivetrain::Init()
{
    // TODO: Set PID gains.

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

    // TODO: shuffleboard.

    // Reset everything
    OKC_CALL(ResetEncoders());
    OKC_CALL(ResetGyro());
    OKC_CALL(ResetDistancePID());
    OKC_CALL(ResetHeadingPID());
    OKC_CALL(ResetTurnPID());

    return true;
}

void Drivetrain::Periodic()
{
    // TODO: implement once network tables and constants are set up
    // It looks like this is just setting PID gains over and over.
}

void Drivetrain::SimulationPeriodic()
{
    // SimulationPeriodic
}

bool Drivetrain::SetSpeedModifier(const double &speed_mod)
{
    speed_modifier_ = speed_mod;

    return true;
}

bool Drivetrain::SetOpenLoopRamp(const double &open_loop_ramp)
{
    OKC_CHECK(interface_->left_motor_1 != nullptr);
    OKC_CHECK(interface_->left_motor_2 != nullptr);
    OKC_CHECK(interface_->left_motor_3 != nullptr);
    OKC_CHECK(interface_->right_motor_1 != nullptr);
    OKC_CHECK(interface_->right_motor_2 != nullptr);
    OKC_CHECK(interface_->right_motor_3 != nullptr);

    // Set the value
    open_loop_ramp_ = open_loop_ramp;

    // Apply the open loop ramp rate to the motors
    interface_->left_motor_1->SetOpenLoopRampRate(open_loop_ramp_);
    interface_->left_motor_2->SetOpenLoopRampRate(open_loop_ramp_);
    interface_->left_motor_3->SetOpenLoopRampRate(open_loop_ramp_);
    interface_->right_motor_1->SetOpenLoopRampRate(open_loop_ramp_);
    interface_->right_motor_2->SetOpenLoopRampRate(open_loop_ramp_);
    interface_->right_motor_3->SetOpenLoopRampRate(open_loop_ramp_);

    return true;
}

bool Drivetrain::CurvatureDrive(const double &speed, const double &turn,
                                const bool turn_in_place)
{
    OKC_CHECK(interface_->diff_drive != nullptr);

    interface_->diff_drive->CurvatureDrive(speed, turn, turn_in_place);

    return true;
}

bool Drivetrain::TankDrive(const double &left_speed, const double &right_speed)
{
    OKC_CHECK(interface_->diff_drive != nullptr);

    interface_->diff_drive->TankDrive(left_speed * speed_modifier_,
                                      right_speed * speed_modifier_, true);

    return true;
}

bool Drivetrain::ArcadeDrive(const double &speed, const double &turn,
                             bool square_inputs)
{
    OKC_CHECK(interface_->diff_drive != nullptr);

    interface_->diff_drive->ArcadeDrive(speed * speed_modifier_,
                                        turn * speed_modifier_, square_inputs);

    return true;
}

bool Drivetrain::ArcadeDriveAuto(const double &speed, const double &turn,
                                 bool square_inputs)
{
    OKC_CALL(ArcadeDrive(speed * -1, turn, square_inputs));

    return true;
}

bool Drivetrain::DriveDistance(const double &distance)
{
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

bool Drivetrain::DriveOnHeading(const double &speed, const double &distance)
{
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

bool Drivetrain::TurnToHeading(const double &heading)
{
    // Set the turn setpoint
    turn_pid_.SetSetpoint(heading);

    // Compute arcade drive inputs
    double cur_heading = 0.0;
    OKC_CALL(this->GetHeading(&cur_heading));
    double cmd_turn = turn_pid_.Calculate(cur_heading);

    OKC_CALL(ArcadeDrive(0, cmd_turn));

    return true;
}

bool Drivetrain::SetHeading()
{
    // From Java code: unclear why this function exists
    // TODO: refactor ported code.

    // Get the current heading and set it to be the setpoint.
    double cur_heading = 0.0;
    OKC_CALL(this->GetHeading(&cur_heading));
    heading_pid_.SetSetpoint(cur_heading);

    return true;
}

bool Drivetrain::GetEncoderAverage(double *avg)
{
    OKC_CHECK(avg != nullptr);

    double left_avg = 0.0;
    OKC_CALL(this->GetLeftEncoderAverage(&left_avg));

    double right_avg = 0.0;
    OKC_CALL(this->GetRightEncoderAverage(&right_avg));

    // Compute the average between the left and right sides.
    *avg = (left_avg + right_avg) / 2.0;

    return true;
}

bool Drivetrain::GetLeftEncoderAverage(double *left_avg)
{
    OKC_CHECK(left_avg != nullptr);
    OKC_CHECK(interface_->left_motor_1 != nullptr);

    // Left encoder 1 is used to represent the whole drivetrain.
    *left_avg = interface_->left_motor_1->GetEncoder().GetPosition();

    return true;
}

bool Drivetrain::GetRightEncoderAverage(double *right_avg)
{
    OKC_CHECK(right_avg != nullptr);
    OKC_CHECK(interface_->right_motor_1 != nullptr);

    // Left encoder 1 is used to represent the whole drivetrain.
    *right_avg = interface_->right_motor_1->GetEncoder().GetPosition();

    return true;
}

bool Drivetrain::ResetEncoders()
{
    OKC_CHECK(interface_->left_motor_1 != nullptr);
    OKC_CHECK(interface_->left_motor_2 != nullptr);
    OKC_CHECK(interface_->left_motor_3 != nullptr);
    OKC_CHECK(interface_->right_motor_1 != nullptr);
    OKC_CHECK(interface_->right_motor_2 != nullptr);
    OKC_CHECK(interface_->right_motor_3 != nullptr);

    // Reset the encoders.
    interface_->left_motor_1->GetEncoder().SetPosition(0.0);
    interface_->left_motor_2->GetEncoder().SetPosition(0.0);
    interface_->left_motor_3->GetEncoder().SetPosition(0.0);
    interface_->right_motor_1->GetEncoder().SetPosition(0.0);
    interface_->right_motor_2->GetEncoder().SetPosition(0.0);
    interface_->right_motor_3->GetEncoder().SetPosition(0.0);

    // TODO: network tables.

    return true;
}

bool Drivetrain::GetInches(const double &rotations, double *inches)
{
    OKC_CHECK(inches != nullptr);

    // TODO: implement once constants and network tables are set up.

    return true;
}

bool Drivetrain::IsAtDistanceSetpoint(bool *at_setpoint)
{
    OKC_CHECK(at_setpoint != nullptr);

    *at_setpoint = dist_pid_.AtSetpoint();

    return true;
}

bool Drivetrain::IsAtHeadingSetpoint(bool *at_setpoint)
{
    OKC_CHECK(at_setpoint != nullptr);

    *at_setpoint = heading_pid_.AtSetpoint();

    return true;
}

bool Drivetrain::IsAtTurnSetpoint(bool *at_setpoint)
{
    OKC_CHECK(at_setpoint != nullptr);

    *at_setpoint = turn_pid_.AtSetpoint();

    return true;
}

bool Drivetrain::ResetDistancePID()
{
    dist_pid_.Reset();
    return true;
}

bool Drivetrain::ResetHeadingPID()
{
    heading_pid_.Reset();
    return true;
}

bool Drivetrain::ResetTurnPID()
{
    turn_pid_.Reset();
    return true;
}

bool Drivetrain::GetHeading(double *heading)
{
    OKC_CHECK(heading != nullptr);
    OKC_CHECK(interface_->ahrs != nullptr);

    *heading = interface_->ahrs->GetAngle();

    return true;
}

bool Drivetrain::ResetGyro()
{
    OKC_CHECK(interface_->ahrs != nullptr);

    interface_->ahrs->Reset();

    return true;
}

bool Drivetrain::SetMaxOutput(const double &max_output)
{
    OKC_CHECK(interface_->diff_drive != nullptr);

    interface_->diff_drive->SetMaxOutput(max_output);

    return true;
}