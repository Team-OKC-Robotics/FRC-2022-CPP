
#pragma once

// TODO: Load all these parameters from a file (JSON/TOML/YAML) so we don't need
// to recompile when they need to be updated.

namespace DrivetrainParams {
    extern double distanceP;
    extern double distanceI;
    extern double distanceD;

    extern double headingP;
    extern double headingI;
    extern double headingD;

    extern double turnP;
    extern double turnI;
    extern double turnD;
} // namespace DrivetrainParams

namespace ShooterParams {
    extern double shootP;
    extern double shootI;
    extern double shootD;
    extern double shootF;

    extern double normalShot;
    extern double againstHub;
    extern double lowGoal;
    extern double farShot;
} // namespace ShooterParams

namespace IntakeParams {
    extern double deployP;
    extern double deployI;
    extern double deployD;
} // namespace IntakeParams

namespace ClimberParams {
    // TODO test and tune the entire climber subsystem
    extern double tiltP;
    extern double tiltD;
    extern double tiltI;

    extern double extendP;
    extern double extendI;
    extern double extendD;
    extern double extendF;
} // namespace ClimberParams

namespace VisionParams {
    // TODO test and tune
    extern double kP;
    extern double kI;
    extern double kD;
    extern double heightOfGoal;   // TODO change this to be the actual value
    extern double heightOfCamera; // TODO change this to be the actual value
    extern double cameraAngle;    // TODO change this to be the actual value
} // namespace VisionParams
