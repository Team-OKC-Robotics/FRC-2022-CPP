#include "Parameters.h"

namespace DrivetrainParams {
    double distanceP = 0.065;
    double distanceI = 0.002;
    double distanceD = 0.001;

    double headingP = 0.07;
    double headingI = 0;
    double headingD = 0.00001;

    double turnP = 0.05;
    double turnI = 0.000;
    double turnD = 0.009;
} // namespace DrivetrainParams

namespace ShooterParams {
    double shootP = 0.00001;
    double shootI = 0;
    double shootD = 0.000005;
    double shootF = 0.4;

    double normalShot = 9000;
    double againstHub = 8000;
    double lowGoal = 1500;
    double farShot = 12000;
} // namespace ShooterParams

namespace IntakeParams {
    double deployP = 0.07;
    double deployI = 0;
    double deployD = 0.001;
} // namespace IntakeParams

namespace ClimberParams {
    // TODO test and tune the entire climber subsystem
    double tiltP = 0;
    double tiltD = 0;
    double tiltI = 0;

    double extendP = 0.00001;
    double extendI = 0;
    double extendD = 0.00000001;
    double extendF = 0;
} // namespace ClimberParams

namespace VisionParams {
    // TODO test and tune
    double kP = 0.03;
    double kI = 0;
    double kD = 0.001;
    double heightOfGoal = 0;   // TODO change this to be the actual value
    double heightOfCamera = 0; // TODO change this to be the actual value
    double cameraAngle = 0;    // TODO change this to be the actual value
} // namespace VisionParams
