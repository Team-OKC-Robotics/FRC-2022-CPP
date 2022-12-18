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
    extern const double neo550TicksPerRev;

    extern const double neoTicksPerRev;
    extern const double falconTicksPerRev;

    // if enabled, disables a lot of the automatic shuffleboard values
    extern const bool competition;
} // namespace RobotConstants

namespace DrivetrainConstants {
    extern const double ticksPerRev;
    extern const double gearRatio;
    extern const double wheelDiameter;

} // namespace DrivetrainConstants

namespace ShooterConstants {} // namespace ShooterConstants

namespace IntakeConstants {
    extern const double gearRatio; // gear ratio on the intake gearbox
    extern const double rotations; // intake needs to rotate 0.4 rotations to
                                   // reach deployed state
    extern const double
        RAISED; // starting position (aka not extended aka raised) is 0
    extern const double EXTENDED; // -rotations / gearRatio
} // namespace IntakeConstants

namespace ClimberConstants {
    extern const double gearRatio;
    extern const double pulleyDiameter; // inches
    extern const double
        extendLength; // 30 * falconTicksPerRev  /  Math.PI * pulleyDiameter;
} // namespace ClimberConstants

namespace VisionConstants {} // namespace VisionConstants