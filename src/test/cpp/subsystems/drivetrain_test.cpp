
#include <gtest/gtest.h>

#include <memory>

#include "io/DrivetrainIO.h"
#include "subsystems/Drivetrain.h"

class DrivetrainTest : public testing::Test {
public:
    virtual void SetUp() {
        // Set up the drivetrain.
        drivetrain_ = std::make_shared<Drivetrain>(&sw_interface_);
    }

protected:
    std::shared_ptr<Drivetrain> drivetrain_;

    DrivetrainSoftwareInterface sw_interface_;
};

TEST_F(DrivetrainTest, InitDrivetrain) {
    // Test drivetrain initialization.
    ASSERT_TRUE(drivetrain_->Init());

    // Check to see that the interface updated
    EXPECT_EQ(sw_interface_.reset_gyro, true);
    EXPECT_EQ(sw_interface_.reset_encoders, true);
    EXPECT_DOUBLE_EQ(sw_interface_.drive_config.open_loop_ramp_rate, 0.01);
    EXPECT_EQ(sw_interface_.update_config, true);
}
