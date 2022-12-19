
#include <stdlib.h>
#include <iostream>

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

/**
 * Test to make sure the intake can be initialized, and test the intake config
 */
TEST_F(IntakeTest, InitIntake) {
    // Test intake initialization.
    ASSERT_TRUE(intake_->Init());

    // Check to see that the interface updated with initial configured values
    EXPECT_DOUBLE_EQ(sw_interface_.intake_config.open_loop_ramp_rate, 0.5);
    EXPECT_DOUBLE_EQ(sw_interface_.intake_config.EXTENDED, 43.75);
    EXPECT_DOUBLE_EQ(sw_interface_.intake_config.RETRACTED, 0);
    EXPECT_DOUBLE_EQ(sw_interface_.intake_config.max_output_deploy, 0.8);
    EXPECT_DOUBLE_EQ(sw_interface_.intake_config.max_output_retract, -0.8);
    EXPECT_DOUBLE_EQ(sw_interface_.intake_config.max_indexer_current, 20);
    EXPECT_EQ(sw_interface_.reset_encoders, true);
    EXPECT_EQ(sw_interface_.update_config, true);
}

/**
 * Test to make sure the intake power (ie intake in cargo and/or spit it out) works
 */
TEST_F(IntakeTest, IntakePowerTest) {
    const double kIntakePower = 1;
    ASSERT_TRUE(intake_->SetIntakePower(kIntakePower));

    EXPECT_EQ(sw_interface_.intake_power, kIntakePower);
}

/**
 * Test to make sure the indexer power (ie intake in cargo and/or spit it out) works
 */
TEST_F(IntakeTest, IndexerPowerTest) {
    const double kIndexerPower = -1;
    ASSERT_TRUE(intake_->SetIndexerPower(kIndexerPower));

    EXPECT_EQ(sw_interface_.indexer_power, kIndexerPower);
}


/**
 * The big test.
 * Tests that the intake position (deploy/extend) logic works
 */
TEST_F(IntakeTest, IntakePositionTest) {
    ASSERT_TRUE(intake_ ->Init());

    double last_intake_output = 0;

    sw_interface_.intake_position_encoder_val = 0; // set the encoder to 0 because it was written to by the last test
    sw_interface_.deployed_limit_switch_val = true; // reset the limit switch too

    // encoder should initially be 0
    EXPECT_DOUBLE_EQ(sw_interface_.intake_position_encoder_val, 0);

    // === EXTEND ===
    ASSERT_TRUE(intake_->SetExtended(true)); // method should not error out
    EXPECT_EQ(intake_->GetDirection(), 1); // internal var 'direction' should be 1
    EXPECT_DOUBLE_EQ(intake_->GetSetpoint(), sw_interface_.intake_config.EXTENDED); // PID setpoint should be the "EXTENDED" constant

    // call periodic so logic updates
    intake_->Periodic();
    intake_->Periodic();

    last_intake_output = sw_interface_.intake_position_power;
    EXPECT_GT(last_intake_output, 0); // should be at least positive

    // set the intake encoder to slightly less than fully extended
    sw_interface_.intake_position_encoder_val = sw_interface_.intake_config.EXTENDED - 0.1;

    // call periodic so logic updates
    intake_->Periodic();
    intake_->Periodic();

    // intake position output now should be less than earlier, because we're closer to the setpoint
    EXPECT_GT(last_intake_output, sw_interface_.intake_position_power);
    last_intake_output = sw_interface_.intake_position_power;

    sw_interface_.deployed_limit_switch_val = false; // deployed limit switch uses inverse logic, so to simulate a press set it to false
    sw_interface_.intake_position_encoder_val = sw_interface_.intake_config.EXTENDED; // cheat and set the value to a known good value this would normally happen automagically but this is a unit test

    // call periodic so logic updates
    intake_->Periodic();

    // and because encoders can get inacurrate, and starting position is never constant
    // the code automatically sets the encoder to the known-good EXTENDED value
    // under normal conditions, but because this is a unit test, we cheat and just go ahead and set 
    // the value ourselves
    EXPECT_EQ(sw_interface_.intake_position_encoder_val, sw_interface_.intake_config.EXTENDED);
    EXPECT_EQ(sw_interface_.intake_position_power, 0); // the limit switch is triggered so the motor should full stop
    
    
    EXPECT_EQ(intake_->IsRetracted(), false);
    EXPECT_EQ(intake_->IsExtended(), true);




    // === RETRACT ===
    ASSERT_TRUE(intake_->SetExtended(false)); // method should not error out
    EXPECT_EQ(intake_->GetDirection(), -1); // internal var 'direction' should be -1
    EXPECT_EQ(intake_->GetSetpoint(), 0); // PID setpoint should be back to 0

    intake_->Periodic(); // call periodic so logic updates

    last_intake_output = sw_interface_.intake_position_power;
    EXPECT_GT(0, last_intake_output); // the output should be negative at least, as the encoder should still read EXTENDED

    // set the intake encoder to slightly more than 0
    sw_interface_.intake_position_encoder_val = 0.1;

    intake_->Periodic(); // call periodic so logic updates

    // intake position output now should be less than earlier, becuase we are closer to the setpoint
    // (remember output is negative now so less negative output > more negative output)
    EXPECT_GT(sw_interface_.intake_position_power, last_intake_output);
    
    // set the encoder to be at our setpoint
    sw_interface_.intake_position_encoder_val = 0;
    // set the deploy limit switch to be unpressed (remember, reverse logic)
    sw_interface_.deployed_limit_switch_val = true;

    // call periodic so logic updates
    intake_->Periodic();

    // and now everything should be all nice and 0 and stuff
    EXPECT_EQ(sw_interface_.intake_position_power, 0);

    EXPECT_EQ(intake_->IsRetracted(), true);
    EXPECT_EQ(intake_->IsExtended(), false);
}