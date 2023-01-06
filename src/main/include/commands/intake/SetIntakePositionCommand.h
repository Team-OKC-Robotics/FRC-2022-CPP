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
class SetIntakePositionCommand
    : public frc2::CommandHelper<frc2::CommandBase, SetIntakePositionCommand> {
public:
    /**
     * Creates a new SetIntakePositionCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    explicit SetIntakePositionCommand(std::shared_ptr<Intake> intake,
                                      bool is_extended);

    void Execute() override;
    bool IsFinished() override;

private:
    std::shared_ptr<Intake> intake_;
    bool is_extended_;
};
