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

// TODO: Load all these constants from a JSON so we don't need to recompile when
// they need to be updated.

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

    const double distanceP = 0.065;
    const double distanceI = 0.002;
    const double distanceD = 0.001;

    const double headingP = 0.07;
    const double headingI = 0;
    const double headingD = 0.00001;

    const double turnP = 0.05;
    const double turnI = 0.000;
    const double turnD = 0.009;
} // namespace DrivetrainConstants

namespace ShooterConstants {
    const double shootP = 0.00001;
    const double shootI = 0;
    const double shootD = 0.000005;
    const double shootF = 0.4;

    const double normalShot = 9000;
    const double againstHub = 8000;
    const double lowGoal = 1500;
    const double farShot = 12000;
} // namespace ShooterConstants

namespace IntakeConstants {
    const double deployP = 0.07;
    const double deployI = 0;
    const double deployD = 0.001;

    const double gearRatio =
        1 / 5 * 1 / 5 * 1 / 5; // gear ratio on the intake gearbox
    const double rotations =
        0.4; // intake needs to rotate 0.4 rotations to reach deployed state

    const double RAISED =
        0; // starting position (aka not extended aka raised) is 0
    const double EXTENDED = 43.75; // -rotations / gearRatio
} // namespace IntakeConstants

namespace ClimberConstants {
    // TODO test and tune the entire climber subsystem
    const double tiltP = 0;
    const double tiltD = 0;
    const double tiltI = 0;

    const double extendP = 0.00001;
    const double extendI = 0;
    const double extendD = 0.00000001;
    const double extendF = 0;

    const double gearRatio = 1 / 625;
    const double pulleyDiameter = 3; // inches
    const double extendLength =
        322281; // 30 * falconTicksPerRev  /  Math.PI * pulleyDiameter;
} // namespace ClimberConstants

namespace VisionConstants {
    // TODO test and tune
    const double kP = 0.03;
    const double kI = 0;
    const double kD = 0.001;
    const double heightOfGoal = 0;   // TODO change this to be the actual value
    const double heightOfCamera = 0; // TODO change this to be the actual value
    const double cameraAngle = 0;    // TODO change this to be the actual value
} // namespace VisionConstants