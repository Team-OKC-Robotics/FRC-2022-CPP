// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Joystick.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/WaitCommand.h>
#include <units/time.h>

#include "Utils.h"
#include "subsystems/Drivetrain.h"

/**
 *
 */
class SetSpeedDrive : public frc2::WaitCommand {
public:
    /**
     * Creates a new DriveCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    explicit SetSpeedDrive(std::shared_ptr<Drivetrain> drivetrain, double speed,
                           units::second_t time)
        : frc2::WaitCommand(time) {
        // Set everything.
        drivetrain_ = drivetrain;
        speed_ = speed;
        time_ = time;

        this->AddRequirements(drivetrain_.get());
    };

    void Execute() override;

    // IsFinished() should be set based on the super-class WaitCommand's
    // IsFinished()

private:
    std::shared_ptr<Drivetrain> drivetrain_;
    double speed_;
    units::second_t time_;
};
