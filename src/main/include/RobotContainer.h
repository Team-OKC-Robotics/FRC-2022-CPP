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
#include "subsystems/Drivetrain.h"
#include "io/SwerveDriveIO.h"
#include "subsystems/SwerveDrive.h"

// Subsystems
#include "subsystems/Drivetrain.h"

// Gamepad
#include "ui/GamepadMap.h"
#include <frc/Joystick.h>
#include <frc2/command/button/JoystickButton.h>

/// Commands
// Drivetrain
#include "commands/drivetrain/DriveCommand.h"
#include "commands/drivetrain/DriveSetSpeedCommand.h"
#include "commands/drivetrain/QuickTeleopDriveCommand.h"
#include "commands/drivetrain/SetSpeedDrive.h"
#include "commands/drivetrain/SlowTeleopDrive.h"
#include "commands/drivetrain/TeleopDriveCommand.h"
#include "commands/drivetrain/TurnCommand.h"
// swerve
#include "commands/swerve/TeleOpSwerveCommand.h"
#include "commands/swerve/AutoSwerveCommand.h"

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
    std::shared_ptr<SwerveDriveIO> swerve_drive_io_;

    // Robot software interfaces.
    std::shared_ptr<DrivetrainSoftwareInterface> drivetrain_sw_;
    std::shared_ptr<SwerveDriveSoftwareInterface> swerve_drive_sw_;

    // Subsystems
    std::shared_ptr<Drivetrain> drivetrain_;
    std::shared_ptr<SwerveDrive> swerve_drive_;

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
    std::shared_ptr<AutoSwerveCommand> m_autonomousCommand;

    // Drivetrain
    std::shared_ptr<QuickTeleopDriveCommand> quick_teleop_drive_command_;
    std::shared_ptr<SlowTeleopDrive> slow_teleop_drive_;
    std::shared_ptr<TeleopDriveCommand> teleop_drive_command_;

    // swerve drive
    std::shared_ptr<TeleOpSwerveCommand> swerve_teleop_command_;
};
