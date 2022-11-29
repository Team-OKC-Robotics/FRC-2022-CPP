// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Errors.h>
#include <frc2/command/Command.h>
#include <frc2/command/SubsystemBase.h>
#include <memory>
#include <vector>

#include "RobotContainer.h"
#include "Utils.h"
#include "commands/ExampleCommand.h"
#include "hardware/HardwareInterface.h"
#include "io/DrivetrainIO.h"
#include "subsystems/Drivetrain.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
public:
    RobotContainer();

    frc2::Command *GetAutonomousCommand();

private:
    // Hardware Initialization
    bool InitHardware(std::unique_ptr<HardwareInterface> &hardware);
    bool InitActuators(ActuatorInterface *actuators_interface);
    bool InitSensors(const ActuatorInterface &actuators,
                     SensorInterface *sensor_interface);

    // Robot Hardware
    std::unique_ptr<HardwareInterface> hardware_;

    // Hardware I/O interfaces
    std::shared_ptr<DrivetrainIO> drivetrain_io_;

    // Robot software interfaces.
    std::shared_ptr<DrivetrainSoftwareInterface> drivetrain_sw_;

    // Subsystems
    std::shared_ptr<Drivetrain> drivetrain_;

    // Commands
    ExampleCommand m_autonomousCommand;

    void ConfigureButtonBindings();
};
