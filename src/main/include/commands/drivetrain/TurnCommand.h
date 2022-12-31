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
class TurnCommand : public frc2::CommandHelper<frc2::CommandBase, TurnCommand> {
public:
    /**
     * Creates a new DriveCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    explicit TurnCommand(std::shared_ptr<Drivetrain> drivetrain, double angle,
                         double max_output = 1.0);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;

private:
    std::shared_ptr<Drivetrain> drivetrain_;
    double angle_;
    double max_output_;
};
