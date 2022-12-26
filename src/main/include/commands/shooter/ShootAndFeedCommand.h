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
class ShootAndFeedCommand : public frc2::WaitCommand {
public:
    /**
     * Creates a new ShootAndFeedCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    explicit ShootAndFeedCommand(std::shared_ptr<Shooter> shooter, double rpm,
                                 units::second_t time)
        : shooter_(shooter), rpm_(rpm), frc2::WaitCommand(time) {
        // Add the shooter as a requirement
        AddRequirements(shooter_.get());
    }

    void Execute() override;

private:
    std::shared_ptr<Shooter> shooter_;
    double rpm_;
};
