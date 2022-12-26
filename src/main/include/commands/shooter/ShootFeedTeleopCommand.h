// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/WaitCommand.h>

#include "Utils.h"
#include "subsystems/Shooter.h"

/**
 * @brief
 *
 */
class ShootFeedTeleopCommand
    : public frc2::CommandHelper<frc2::CommandBase, ShootFeedTeleopCommand> {
public:
    /**
     * Creates a new ShootFeedTeleopCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    explicit ShootFeedTeleopCommand(std::shared_ptr<Shooter> shooter,
                                    double rpm, double power)
        : shooter_(shooter), rpm_(rpm), power_(power) {

        // Add the shooter as a requirement
        if (shooter_ != nullptr) {
            this->AddRequirements(shooter_.get());
        }
    }

    void Execute() override;
    void End(bool executed) override;
    bool IsFinished() override;

private:
    std::shared_ptr<Shooter> shooter_;
    double rpm_;
    double power_;
};
