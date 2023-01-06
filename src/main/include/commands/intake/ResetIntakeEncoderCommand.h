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
class ResetIntakeEncoderCommand
    : public frc2::CommandHelper<frc2::CommandBase, ResetIntakeEncoderCommand> {
public:
    /**
     * Creates a new ResetIntakeEncoderCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    explicit ResetIntakeEncoderCommand(std::shared_ptr<Intake> intake);

    void Execute() override;
    bool IsFinished() override;

private:
    std::shared_ptr<Intake> intake_;
};
