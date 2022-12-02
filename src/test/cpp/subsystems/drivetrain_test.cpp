
#include <gtest/gtest.h>

#include <memory>

#include "io/DrivetrainIO.h"
#include "subsystems/Drivetrain.h"

class DrivetrainTest : public testing::Test {
public:
    virtual void SetUp() {
        // Set up the drivetrain.
        drivetrain_ = std::make_shared<Drivetrain>(&sw_interface_);

        // Set the speed modifier to the default for the Drivetrain.
        speed_mod_ = 0.75;

        // Initialize the drivetrain.
        ASSERT_TRUE(drivetrain_->Init());
    }

protected:
    std::shared_ptr<Drivetrain> drivetrain_;

    double speed_mod_;

    DrivetrainSoftwareInterface sw_interface_;
};

TEST_F(DrivetrainTest, InitDrivetrainTest) {
    // Check to see that the interface updated when the drivetrain was
    // initialized.
    EXPECT_EQ(sw_interface_.reset_gyro, true);
    EXPECT_EQ(sw_interface_.reset_encoders, true);
    EXPECT_DOUBLE_EQ(sw_interface_.drive_config.open_loop_ramp_rate, 0.01);
    EXPECT_EQ(sw_interface_.update_config, true);
}

TEST_F(DrivetrainTest, SetOpenLoopRampTest) {
    const double kRamp = 0.9;
    ASSERT_TRUE(drivetrain_->SetOpenLoopRamp(kRamp));
    EXPECT_DOUBLE_EQ(sw_interface_.drive_config.open_loop_ramp_rate, kRamp);
    EXPECT_TRUE(sw_interface_.update_config);
}

TEST_F(DrivetrainTest, CurvatureDriveTest) {
    const double kPower = 1.0;
    const double kTurn = 0.8;
    const bool kTurnInPlace = true;
    ASSERT_TRUE(drivetrain_->CurvatureDrive(kPower, kTurn, kTurnInPlace));

    EXPECT_EQ(sw_interface_.drive_mode, DriveMode::CURVATURE);
    EXPECT_DOUBLE_EQ(sw_interface_.arcade_power, kPower);
    EXPECT_DOUBLE_EQ(sw_interface_.arcade_turn, kTurn);
    EXPECT_EQ(sw_interface_.turn_in_place, kTurnInPlace);
}

TEST_F(DrivetrainTest, TankDriveTest) {
    const double kLeft = 1.0;
    const double kRight = 0.8;
    ASSERT_TRUE(drivetrain_->TankDrive(kLeft, kRight));

    EXPECT_EQ(sw_interface_.drive_mode, DriveMode::TANK);
    EXPECT_DOUBLE_EQ(sw_interface_.tank_left, kLeft * speed_mod_);
    EXPECT_DOUBLE_EQ(sw_interface_.tank_right, kRight * speed_mod_);
    EXPECT_EQ(sw_interface_.square_inputs, true);
}

TEST_F(DrivetrainTest, ArcadeDriveTest) {
    const double kPower = 1.0;
    const double kTurn = 0.8;
    const bool kSquareInputs = true;
    ASSERT_TRUE(drivetrain_->ArcadeDrive(kPower, kTurn, kSquareInputs));

    EXPECT_EQ(sw_interface_.drive_mode, DriveMode::ARCADE);
    EXPECT_DOUBLE_EQ(sw_interface_.arcade_power, kPower * speed_mod_);
    EXPECT_DOUBLE_EQ(sw_interface_.arcade_turn, kTurn * speed_mod_);
    EXPECT_EQ(sw_interface_.square_inputs, kSquareInputs);
}

TEST_F(DrivetrainTest, ArcadeDriveAutoTest) {
    const double kPower = 1.0;
    const double kTurn = 0.8;
    const bool kSquareInputs = false;
    ASSERT_TRUE(drivetrain_->ArcadeDriveAuto(kPower, kTurn, kSquareInputs));

    EXPECT_EQ(sw_interface_.drive_mode, DriveMode::ARCADE);
    EXPECT_DOUBLE_EQ(sw_interface_.arcade_power, -1 * kPower);
    EXPECT_DOUBLE_EQ(sw_interface_.arcade_turn, kTurn);
    EXPECT_EQ(sw_interface_.square_inputs, kSquareInputs);
}

TEST_F(DrivetrainTest, SpeedModifierTest) {
    speed_mod_ = 0.5;
    ASSERT_TRUE(drivetrain_->SetSpeedModifier(speed_mod_));
    // TODO: getter for speed modifier.
}

TEST_F(DrivetrainTest, EncoderAverageTests) {
    const double kLeftEncoder = 50.0;
    const double kRightEncoder = 49.0;
    const double kEncoderAverage = 49.5;

    sw_interface_.left_encoder_avg = kLeftEncoder;
    sw_interface_.right_encoder_avg = kRightEncoder;

    // Ensure encoder averages fail for nullptr.
    ASSERT_FALSE(drivetrain_->GetEncoderAverage(nullptr));
    ASSERT_FALSE(drivetrain_->GetLeftEncoderAverage(nullptr));
    ASSERT_FALSE(drivetrain_->GetRightEncoderAverage(nullptr));

    // Get encoder averages with legitimate pointers.
    double avg = 0.0;
    ASSERT_TRUE(drivetrain_->GetLeftEncoderAverage(&avg));
    EXPECT_DOUBLE_EQ(avg, kLeftEncoder);

    ASSERT_TRUE(drivetrain_->GetRightEncoderAverage(&avg));
    EXPECT_DOUBLE_EQ(avg, kRightEncoder);

    ASSERT_TRUE(drivetrain_->GetEncoderAverage(&avg));
    EXPECT_DOUBLE_EQ(avg, kEncoderAverage);
}

TEST_F(DrivetrainTest, ResetEncodersTest) {
    ASSERT_TRUE(drivetrain_->ResetEncoders());
    EXPECT_TRUE(sw_interface_.reset_encoders);
}

// TODO: GetInchesTest once GetInches is implemented.
// TEST_F(DrivetrainTest, GetInchesTest) {
//     const double kRotations = 50.0;
//     const double kInches =
// }

TEST_F(DrivetrainTest, GetHeadingTest) {
    // Ensure this fails for nullptr.
    ASSERT_FALSE(drivetrain_->GetHeading(nullptr));

    // Spoof the IMU.
    const double kHeading = 90.0;
    sw_interface_.imu_yaw = kHeading;

    // For valid pointer, get the heading.
    double hdg = 0.0;
    ASSERT_TRUE(drivetrain_->GetHeading(&hdg));
    EXPECT_EQ(hdg, kHeading);
}

TEST_F(DrivetrainTest, ResetGyroTest) {
    ASSERT_TRUE(drivetrain_->ResetGyro());
    EXPECT_TRUE(sw_interface_.reset_gyro);
}

TEST_F(DrivetrainTest, SetMaxOutputTest) {
    const double kMaxOutput = 0.9;
    ASSERT_TRUE(drivetrain_->SetMaxOutput(kMaxOutput));
    EXPECT_DOUBLE_EQ(sw_interface_.drive_config.max_output, kMaxOutput);
    EXPECT_TRUE(sw_interface_.update_config);
}
