
#include "hardware/HardwareInterface.h"

bool SetupDrivetrainInterface(
    std::unique_ptr<HardwareInterface> &hardware,
    std::shared_ptr<DrivetrainHardwareInterface> *interface) {
    OKC_CHECK(interface != nullptr);
    OKC_CHECK(hardware->actuators != nullptr);
    OKC_CHECK(hardware->sensors != nullptr);

    // Get actuators interface for drivetrain.
    std::unique_ptr<ActuatorInterface> &actuators = hardware->actuators;
    std::unique_ptr<SensorInterface> &sensors = hardware->sensors;

    // Build motor control groups and differential drive.
    frc::MotorControllerGroup left_motors{*actuators->left_motor_1,
                                          *actuators->left_motor_2,
                                          *actuators->left_motor_3};

    frc::MotorControllerGroup right_motors{*actuators->right_motor_1,
                                           *actuators->right_motor_2,
                                           *actuators->right_motor_3};
    // motors face opposite directions so +1 for left side is opposite +1 for
    // right side, this fixes that
    right_motors.SetInverted(true);

    // Ensure the differential drivetrain is un-initialized.
    OKC_CHECK(hardware->diff_drive == nullptr);

    // Create the differential drive.
    hardware->diff_drive =
        std::make_unique<frc::DifferentialDrive>(left_motors, right_motors);

    // Protect against nullptr actuators/sensors
    OKC_CHECK(actuators->left_motor_1 != nullptr);
    OKC_CHECK(actuators->left_motor_2 != nullptr);
    OKC_CHECK(actuators->left_motor_3 != nullptr);
    OKC_CHECK(actuators->right_motor_1 != nullptr);
    OKC_CHECK(actuators->right_motor_2 != nullptr);
    OKC_CHECK(actuators->right_motor_3 != nullptr);
    OKC_CHECK(hardware->diff_drive != nullptr);
    OKC_CHECK(sensors->ahrs != nullptr);

    // Set up drivetrain interface.
    DrivetrainHardwareInterface drivetrain_interface = {
        actuators->left_motor_1.get(),  actuators->left_motor_2.get(),
        actuators->left_motor_3.get(),  actuators->right_motor_1.get(),
        actuators->right_motor_2.get(), actuators->right_motor_3.get(),
        hardware->diff_drive.get(),     hardware->sensors->ahrs.get()};

    // Set the output interface
    *interface =
        std::make_shared<DrivetrainHardwareInterface>(drivetrain_interface);

    return true;
}

bool SetupSwerveDriveInterface(
    std::unique_ptr<HardwareInterface> &hardware,
    std::shared_ptr<SwerveDriveHardwareInterface> *interface) {
    OKC_CHECK(interface != nullptr);
    OKC_CHECK(hardware->actuators != nullptr);
    OKC_CHECK(hardware->sensors != nullptr);

    // Get actuators interface for swerve drive.
    std::unique_ptr<ActuatorInterface> &actuators = hardware->actuators;
    std::unique_ptr<SensorInterface> &sensors = hardware->sensors;

    // make sure all the actuators actually exist
    OKC_CHECK(actuators->left_front_drive_motor != nullptr);
    OKC_CHECK(actuators->left_back_drive_motor != nullptr);
    OKC_CHECK(actuators->right_front_drive_motor != nullptr);
    OKC_CHECK(actuators->right_back_drive_motor != nullptr);
    OKC_CHECK(actuators->left_front_steer_motor != nullptr);
    OKC_CHECK(actuators->left_back_steer_motor != nullptr);
    OKC_CHECK(actuators->right_front_steer_motor != nullptr);
    OKC_CHECK(actuators->right_back_steer_motor != nullptr);

    OKC_CHECK(sensors->ahrs != nullptr);
    OKC_CHECK(sensors->left_front_steer_encoder != nullptr);
    OKC_CHECK(sensors->left_back_steer_encoder != nullptr);
    OKC_CHECK(sensors->right_front_steer_encoder != nullptr);
    OKC_CHECK(sensors->right_back_steer_encoder != nullptr);


    // set up swerve drive interface.
    SwerveDriveHardwareInterface swerve_drive_interface = {
        actuators->left_front_drive_motor.get(),
        actuators->left_back_drive_motor.get(),
        actuators->right_front_drive_motor.get(),
        actuators->right_back_drive_motor.get(),
        actuators->left_front_steer_motor.get(),
        actuators->left_back_steer_motor.get(),
        actuators->right_front_steer_motor.get(),
        actuators->right_back_steer_motor.get(),

        sensors->ahrs.get(),

        sensors->left_front_drive_encoder.get(),
        sensors->left_back_drive_encoder.get(),
        sensors->right_front_drive_encoder.get(),
        sensors->right_back_drive_encoder.get(),

        sensors->left_front_steer_encoder.get(),
        sensors->left_back_steer_encoder.get(),
        sensors->right_front_steer_encoder.get(),
        sensors->right_back_steer_encoder.get(),

        sensors->left_front_steer_vel_encoder.get(),
        sensors->left_back_steer_vel_encoder.get(),
        sensors->right_front_steer_vel_encoder.get(),
        sensors->right_back_steer_vel_encoder.get(),
    };

    // set the output interface
    *interface = std::make_shared<SwerveDriveHardwareInterface>(swerve_drive_interface);

    return true;
}

bool SetupIntakeInterface(std::unique_ptr<HardwareInterface> &hardware,
                          std::shared_ptr<IntakeHardwareInterface> *interface) {
    OKC_CHECK(interface != nullptr);
    OKC_CHECK(hardware->actuators != nullptr);
    OKC_CHECK(hardware->sensors != nullptr);

    // Get actuators interface for intake.
    std::unique_ptr<ActuatorInterface> &actuators = hardware->actuators;
    std::unique_ptr<SensorInterface> &sensors = hardware->sensors;

    // Protect against nullptr actuators/sensors
    OKC_CHECK(actuators->intake_position_motor != nullptr);
    OKC_CHECK(actuators->intake_motor != nullptr);
    OKC_CHECK(actuators->indexer_motor != nullptr);
    OKC_CHECK(sensors->retracted_limit_switch != nullptr);
    OKC_CHECK(sensors->deploy_limit_switch != nullptr);

    // Set up intake interface.
    IntakeHardwareInterface intake_interface = {
        actuators->intake_position_motor.get(),
        actuators->intake_motor.get(),
        actuators->indexer_motor.get(),

        sensors->retracted_limit_switch.get(),
        sensors->deploy_limit_switch.get(),
    };

    // Set the output interface
    *interface = std::make_shared<IntakeHardwareInterface>(intake_interface);

    return true;
}

bool SetupShooterInterface(
    std::unique_ptr<HardwareInterface> &hardware,
    std::shared_ptr<ShooterHardwareInterface> *interface) {
    OKC_CHECK(interface != nullptr);
    OKC_CHECK(hardware->actuators != nullptr);
    OKC_CHECK(hardware->sensors != nullptr);

    // Get actuators and sensors interfaces for shooter.
    std::unique_ptr<ActuatorInterface> &actuators = hardware->actuators;
    std::unique_ptr<SensorInterface> &sensors = hardware->sensors;

    // Protect against nullptr actuators/sensors
    OKC_CHECK(actuators->shooter_motor != nullptr);
    OKC_CHECK(actuators->trigger_motor != nullptr);
    OKC_CHECK(sensors->ball_detector != nullptr);

    ShooterHardwareInterface shooter_interface = {
        actuators->shooter_motor.get(), actuators->trigger_motor.get(),
        sensors->ball_detector.get()};

    // Set the output interface
    *interface = std::make_shared<ShooterHardwareInterface>(shooter_interface);

    return true;
}