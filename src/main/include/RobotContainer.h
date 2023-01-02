// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Errors.h>

#include <memory>
#include <vector>

#include "Parameters.h"
#include "RobotContainer.h"
#include "Utils.h"

// Hardware
#include "hardware/HardwareInterface.h"

// I/O Subsystems
#include "io/DrivetrainIO.h"
#include "io/IntakeIO.h"
#include "io/ShooterIO.h"

// Subsystems
#include "subsystems/Drivetrain.h"
#include "subsystems/Intake.h"
#include "subsystems/Shooter.h"

// Gamepad
#include "ui/GamepadMap.h"
#include <frc/Joystick.h>
#include <frc2/command/button/JoystickButton.h>

/// Commands
#include "commands/ExampleCommand.h"
// Drivetrain
#include "commands/drivetrain/DriveCommand.h"
#include "commands/drivetrain/DriveSetSpeedCommand.h"
#include "commands/drivetrain/QuickTeleopDriveCommand.h"
#include "commands/drivetrain/SetSpeedDrive.h"
#include "commands/drivetrain/SlowTeleopDrive.h"
#include "commands/drivetrain/TeleopDriveCommand.h"
#include "commands/drivetrain/TurnCommand.h"
// Shooter
#include "commands/shooter/FeedCommand.h"
#include "commands/shooter/SetTriggerCommand.h"
#include "commands/shooter/ShooterPresetCommand.h"
#include "commands/shooter/StopShooterCommand.h"
// Intake
#include "commands/intake/SetIndexerCommand.h"
#include "commands/intake/SetIntakeCommand.h"
#include "commands/intake/SetIntakePositionCommand.h"

#include <frc2/command/Command.h>
#include <frc2/command/SubsystemBase.h>

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

    std::shared_ptr<frc2::Command> GetAutonomousCommand();
    std::shared_ptr<frc2::Command> GetDriveCommand();

private:
    // Hardware Initialization
    bool InitHardware(std::unique_ptr<HardwareInterface> &hardware);
    bool InitActuators(ActuatorInterface *actuators_interface);
    bool InitSensors(const ActuatorInterface &actuators,
                     SensorInterface *sensor_interface);

    // Command initialization
    bool InitCommands();

    // Gamepad initialization
    bool InitGamepads();
    void ConfigureButtonBindings();

    // Robot Hardware
    std::unique_ptr<HardwareInterface> hardware_;

    // Hardware I/O interfaces
    std::shared_ptr<DrivetrainIO> drivetrain_io_;
    std::shared_ptr<IntakeIO> intake_io_;
    std::shared_ptr<ShooterIO> shooter_io_;

    // Robot software interfaces.
    std::shared_ptr<DrivetrainSoftwareInterface> drivetrain_sw_;
    std::shared_ptr<IntakeSoftwareInterface> intake_sw_;
    std::shared_ptr<ShooterSoftwareInterface> shooter_sw_;

    // Subsystems
    std::shared_ptr<Drivetrain> drivetrain_;
    std::shared_ptr<Intake> intake_;
    std::shared_ptr<Shooter> shooter_;

    /**
     * User interfaces
     * - Gamepads
     * - Joystick Buttons
     */
    std::shared_ptr<frc::Joystick> gamepad1_;
    std::shared_ptr<frc::Joystick> gamepad2_;

    std::shared_ptr<frc2::JoystickButton> driver_a_button_;
    std::shared_ptr<frc2::JoystickButton> driver_b_button_;
    std::shared_ptr<frc2::JoystickButton> driver_back_button_;
    std::shared_ptr<frc2::JoystickButton> driver_left_bumper_;
    std::shared_ptr<frc2::JoystickButton> driver_right_bumper_;

    std::shared_ptr<frc2::JoystickButton> manip_a_button_;
    std::shared_ptr<frc2::JoystickButton> manip_b_button_;
    std::shared_ptr<frc2::JoystickButton> manip_back_button_;
    std::shared_ptr<frc2::JoystickButton> manip_start_button_;
    std::shared_ptr<frc2::JoystickButton> manip_left_stick_button_;
    std::shared_ptr<frc2::JoystickButton> manip_right_stick_button_;

    /**
     * Commands
     */
    std::shared_ptr<ExampleCommand> m_autonomousCommand;

    // Drivetrain
    std::shared_ptr<QuickTeleopDriveCommand> quick_teleop_drive_command_;
    std::shared_ptr<SlowTeleopDrive> slow_teleop_drive_;
    std::shared_ptr<TeleopDriveCommand> teleop_drive_command_;

    // Shooter
    std::shared_ptr<FeedCommand> feed_command_;
    std::shared_ptr<StopShooterCommand> stop_shooter_command_;
    std::shared_ptr<SetTriggerCommand> stop_trigger_command_;
    std::shared_ptr<ShooterPresetCommand> shooter_preset_command_;
    std::shared_ptr<SetTriggerCommand> trigger_in_;
    std::shared_ptr<SetTriggerCommand> trigger_out_;

    // Intake
    std::shared_ptr<SetIndexerCommand> indexer_in_;
    std::shared_ptr<SetIndexerCommand> indexer_out_;
    std::shared_ptr<SetIndexerCommand> stop_indexer_;
    std::shared_ptr<SetIntakeCommand> intake_in_;
    std::shared_ptr<SetIntakeCommand> intake_out_;
    std::shared_ptr<SetIntakeCommand> stop_intake_;
    std::shared_ptr<SetIntakePositionCommand> deploy_intake_;
    std::shared_ptr<SetIntakePositionCommand> retract_intake_;
};
