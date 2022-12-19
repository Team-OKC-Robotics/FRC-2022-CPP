
#include <gtest/gtest.h>

#include <memory>

#include "io/IntakeIO.h"
#include "subsystems/Intake.h"

class IntakeTest : public testing::Test {
public:
    virtual void SetUp() {
        // Set up the intake.
        intake_ = std::make_shared<Intake>(&sw_interface_);
    }

protected:
    std::shared_ptr<Intake> intake_;

    IntakeSoftwareInterface sw_interface_;
};

TEST_F(IntakeTest, InitIntake) {
    // Test intake initialization.
    ASSERT_TRUE(intake_->Init());

    // Check to see that the interface updated
    EXPECT_EQ(sw_interface_.reset_encoders, true);
    EXPECT_DOUBLE_EQ(sw_interface_.intake_config.open_loop_ramp_rate, 0.5);
    EXPECT_EQ(sw_interface_.update_config, true);
}

TEST_F(IntakeTest, IntakePowerTest) {
    const double kIntakePower = 1;
    ASSERT_TRUE(intake_->SetIntakePower(kIntakePower));

    EXPECT_EQ(sw_interface_.intake_power, kIntakePower);
}

TEST_F(IntakeTest, IndexerPowerTest) {
    const double kIndexerPower = -1;
    ASSERT_TRUE(intake_->SetIndexerPower(kIndexerPower));

    EXPECT_EQ(sw_interface_.indexer_power, kIndexerPower);
}

/**
 * The big test.
 * This needs to perform/test the following:
 *  - initial encoder position should be 0
 *  - tell intake to deploy
 *  - direction should be 1
 *  - intakePID setpoint should be some constant
 *  - call periodic()
 *  - intake position motor power should be some positive(?) value
 *  - set intake position encoder to some number between 0 and DEPLOYED
 *  - call periodic()
 *  - intake position motor power should be a smaller (abs() at least) value
 *  - set intake deployed limit switch to true
 *  - call periodic()
 *  - intake position motor power should be 0
 *  - intake position encoder should be DEPLOYED
 *  - IsExtended should be true()
 * 
 * 
 *  - tell intake to retract
 *  - call periodic()
 *  - direction should be -1
 *  - intake position motor power should be some value of the opposite sign of the value for deploying
 *  - set position encoder to some value between 0 and DEPLOYED
 *  - call periodic()
 *  - intake position motor power should be a smaller value
 *  - set position encoder to 0
 *  - intake position motor power should be 0
 *  - IsExtended() should be false
 */
TEST_F(IntakeTest, IntakePositionTest) {
    double last_intake_output = 0;

    // encoder should initially be 0
    EXPECT_DOUBLE_EQ(sw_interface_.intake_position_encoder_val, 0);

    // === EXTEND ===

    ASSERT_TRUE(intake_->SetIntakePosition(true)); // method should not error out
    EXPECT_EQ(intake_->GetDirection(), 1); // internal var 'direction' should be 1
    EXPECT_DOUBLE_EQ(intake_->GetSetpoint(), sw_interface_.EXTENDED); // PID setpoint should be the "EXTENDED" constant

    intake_->Periodic(); // call periodic so logic updates
    last_intake_output = sw_interface_.intake_position_power
    EXPECT_EQ(last_intake_output > 0, true); // I think the output should be positive at least

    // set the intake encoder to slightly less than extended
    sw_interface_.intake_position_encoder_val = sw_interface_.EXTENDED - 1;

    intake_->Periodic(); // call periodic so logic updates

    // intake position output now should be less than earlier, becuase we are closer to the setpoint
    EXPECT_EQ(sw_interface_.intake_position_power < last_intake_output, true);
    last_intake_output = sw_interface_.intake_position_power;

    sw_interface_.deployed_limit_switch = false; // deployed limit switch uses inverse logic, so to simulate a press set it to false

    intake_->Periodic(); // call periodic so logic updates

    EXPECT_EQ(sw_interface_.intake_position_power, 0); // the limit switch is triggered so the motor should full stop
    EXPECT_EQ(sw_interface_.intake_position_encoder_val, sw_interface_.EXTENDED); // and because encoders can get inacurrate, and starting position is never constant
                                                                                  // the code automatically sets the encoder to the known-good EXTENDED value
    
    // === RETRACT ===
    ASSERT_TRUE(intake_->SetIntakePosition(false)); // method should not error out
    EXPECT_EQ(intake_->GetDirection(), -1); // internal var 'direction' should be -1
    EXPECT_DOUBLE_EQ(intake_->GetSetpoint(), 0); // PID setpoint should be back to 0

    intake_->Periodic(); // call periodic so logic updates
    last_intake_output = sw_interface_.intake_position_power
    EXPECT_EQ(last_intake_output < 0, true); // I think the output should be negative at least, as the encoder should still read EXTENDED

    // set the intake encoder to slightly more than 0
    sw_interface_.intake_position_encoder_val = 1;

    intake_->Periodic(); // call periodic so logic updates

    // intake position output now should be less than earlier, becuase we are closer to the setpoint
    // (remember output is negative now so less negative output > more negative output)
    EXPECT_EQ(sw_interface_.intake_position_power > last_intake_output, true);
    
    // set the encoder to be at our setpoint
    sw_interface_.intake_position_encoder_val = 0;

    // call periodic so logic updates
    intake_->Periodic();

    // and now everything should be all nice and 0 and stuff
    EXPECT_EQ(sw_interface_.intake_position_power, 0);
}