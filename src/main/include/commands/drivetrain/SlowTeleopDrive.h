// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Joystick.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Utils.h"
#include "subsystems/Drivetrain.h"

/**
 *
 */
class SlowTeleopDrive
    : public frc2::CommandHelper<frc2::CommandBase, SlowTeleopDrive> {
public:
    /**
     * Creates a new DriveCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    explicit SlowTeleopDrive(std::shared_ptr<Drivetrain> drivetrain,
                             std::shared_ptr<frc::Joystick> gamepad);

    void Execute() override;
    bool IsFinished() override;

private:
    std::shared_ptr<Drivetrain> drivetrain_;
    std::shared_ptr<frc::Joystick> gamepad_;
};