// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

// TODO: Load all these parameters from a file (JSON/TOML/YAML) so we don't need
// to recompile when they need to be updated.

namespace RobotConstants {
    // although I think getPosition() might return in native units of
    // rotations so this would then be 1
    const double neo550TicksPerRev = 42;

    const double neoTicksPerRev = 42;
    const double falconTicksPerRev = 2048;

    // if enabled, disables a lot of the automatic shuffleboard values
    const bool competition = false;
} // namespace RobotConstants

namespace DrivetrainConstants {
    const double ticksPerRev = neoTicksPerRev;
    const double gearRatio = 1 / 9.64;
    const double wheelDiameter = 6;

} // namespace DrivetrainConstants

namespace ShooterConstants {} // namespace ShooterConstants

namespace IntakeConstants {
    const double gearRatio =
        1 / 5 * 1 / 5 * 1 / 5; // gear ratio on the intake gearbox
    const double rotations =
        0.4; // intake needs to rotate 0.4 rotations to reach deployed state

    const double RAISED =
        0; // starting position (aka not extended aka raised) is 0
    const double EXTENDED = 43.75; // -rotations / gearRatio
} // namespace IntakeConstants

namespace ClimberConstants {
    const double gearRatio = 1 / 625;
    const double pulleyDiameter = 3; // inches
    const double extendLength =
        322281; // 30 * falconTicksPerRev  /  Math.PI * pulleyDiameter;
} // namespace ClimberConstants

namespace VisionConstants {} // namespace VisionConstants