// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Utils.h"
#include "subsystems/Shooter.h"

/**
 * @brief
 *
 */
class SetShooterCommand
    : public frc2::CommandHelper<frc2::CommandBase, SetShooterCommand> {
public:
    /**
     * Creates a new SetShooterCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    explicit SetShooterCommand(std::shared_ptr<Shooter> shooter, double rpm);

    void Execute() override;
    bool IsFinished() override;

private:
    std::shared_ptr<Shooter> shooter_;
    double rpm_;
};
