// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Joystick.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Utils.h"
#include "subsystems/Shooter.h"

/**
 * @brief
 *
 */
class StopShooterCommand
    : public frc2::CommandHelper<frc2::CommandBase, StopShooterCommand> {
public:
    /**
     * Creates a new StopShooterCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    explicit StopShooterCommand(std::shared_ptr<Shooter> shooter)
        : shooter_(shooter) {

        // Add the shooter as a requirement
        AddRequirements(shooter_.get());
    }

    void Execute() override;
    bool IsFinished() override;

private:
    std::shared_ptr<Shooter> shooter_;
};
