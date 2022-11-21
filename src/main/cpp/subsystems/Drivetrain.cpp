
#include "subsystems/Drivetrain.h"

void Drivetrain::Periodic()
{
    // Periodic
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
    OKC_CHECK(interface_->diff_drive != nullptr);

    interface_->diff_drive->ArcadeDrive(speed * -1, turn, square_inputs);

    return true;
}

bool Drivetrain::DriveDistance(const double &distance)
{
    OKC_CHECK(interface_->diff_drive != nullptr);

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

    interface_->diff_drive->ArcadeDrive(dist_pid_.Calculate(current_dist),
                                        heading_pid_.Calculate(heading));

    return true;
}

bool Drivetrain::DriveOnHeading(const double &speed, const double &distance)
{
    OKC_CHECK(interface_->diff_drive != nullptr);

    // Set the setpoint.
    dist_pid_.SetSetpoint(distance);

    return true;
}
