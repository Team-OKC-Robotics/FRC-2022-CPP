// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Utils.h"
#include "subsystems/Intake.h"

/**
 *
 */
class SetIndexerCommand
    : public frc2::CommandHelper<frc2::CommandBase, SetIndexerCommand> {
public:
    /**
     * Creates a new SetIndexerCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    explicit SetIndexerCommand(std::shared_ptr<Intake> intake, double power);

    void Execute() override;
    bool IsFinished() override;

private:
    std::shared_ptr<Intake> intake_;
    double power_;
};
