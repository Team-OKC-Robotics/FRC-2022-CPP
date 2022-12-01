
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
    // Test drivetrain initialization.
    ASSERT_TRUE(intake_->Init());

    // Check to see that the interface updated
    EXPECT_EQ(sw_interface_.reset_encoders, true);
    EXPECT_DOUBLE_EQ(sw_interface_.drive_config.open_loop_ramp_rate, 0.01);
    EXPECT_EQ(sw_interface_.update_config, true);
}
