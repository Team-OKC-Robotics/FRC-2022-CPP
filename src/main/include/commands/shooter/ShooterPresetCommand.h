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
class ShooterPresetCommand
    : public frc2::CommandHelper<frc2::CommandBase, ShooterPresetCommand> {
public:
    /**
     * Creates a new ShooterPresetCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    explicit ShooterPresetCommand(std::shared_ptr<Shooter> shooter,
                                  std::shared_ptr<frc::Joystick> gamepad,
                                  double power)
        : shooter_(shooter), gamepad_(gamepad), power_(power) {

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
    std::shared_ptr<frc::Joystick> gamepad_;
    double power_;
};
