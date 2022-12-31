
#include <gtest/gtest.h>
#include <memory>

#include "io/ShooterIO.h"
#include "subsystems/Shooter.h"

class ShooterTest : public testing::Test {
public:
    virtual void SetUp() {
        // Set up the intake.
        shooter_ = std::make_shared<Shooter>(&sw_interface_);
        shooter_->Init();
    }

protected:
    std::shared_ptr<Shooter> shooter_;

    ShooterSoftwareInterface sw_interface_;
};

TEST_F(ShooterTest, ResetPowerTest) {
    // Initialize the shooter power to non-zero
    sw_interface_.shooter_power = 0.5;

    // Reset the power
    ASSERT_TRUE(shooter_->ResetPower());

    // Verify that the power is now 0
    EXPECT_DOUBLE_EQ(sw_interface_.shooter_power, 0.0);
}

TEST_F(ShooterTest, StopShooterTest) {
    // Initialize the shooter to not stopped.
    sw_interface_.stop_shooter = false;

    // Stop the shooter
    ASSERT_TRUE(shooter_->StopShooter());

    // Verify the shooter is stopped.
    EXPECT_TRUE(sw_interface_.stop_shooter);
}

TEST_F(ShooterTest, SetTriggerTest) {
    // Initialize the trigger power
    sw_interface_.trigger_power = 0.0;
    sw_interface_.is_ball_detected = false;

    // Test that the trigger power is set to the specified value with no ball
    // detected.
    const double kTriggerPower = 0.5;
    ASSERT_TRUE(shooter_->SetTrigger(kTriggerPower));

    // Verify the correct power is set.
    EXPECT_DOUBLE_EQ(sw_interface_.trigger_power, kTriggerPower);

    // Now do the same thing, but with a ball detected
    sw_interface_.is_ball_detected = true;

    // We expect that the power is set to 0 with a ball since this is a positive
    // power. We don't allow the trigger to shoot unless Feed() is called.
    ASSERT_TRUE(shooter_->SetTrigger(kTriggerPower));
    EXPECT_DOUBLE_EQ(sw_interface_.trigger_power, 0.0);

    // Now send a negative power (backward)
    const double kBackwardTrigger = -0.5;
    ASSERT_TRUE(shooter_->SetTrigger(kBackwardTrigger));
    EXPECT_DOUBLE_EQ(sw_interface_.trigger_power, kBackwardTrigger);
}

TEST_F(ShooterTest, FeedTest) {
    // Initialize the trigger to 0 power.
    sw_interface_.trigger_power = 0.0;

    // Command a feed.
    const double kFeedPower = 1.0;
    ASSERT_TRUE(shooter_->Feed(kFeedPower));

    // Verify that the feed command happened.
    EXPECT_DOUBLE_EQ(sw_interface_.trigger_power, kFeedPower);
}
