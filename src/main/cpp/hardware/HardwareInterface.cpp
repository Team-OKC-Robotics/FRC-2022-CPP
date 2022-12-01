
#include "hardware/HardwareInterface.h"

bool SetupDrivetrainInterface(
    std::unique_ptr<HardwareInterface> &hardware,
    std::shared_ptr<DrivetrainHardwareInterface> *interface) {
    OKC_CHECK(interface != nullptr);
    OKC_CHECK(hardware->actuators != nullptr);
    OKC_CHECK(hardware->sensors != nullptr);

    // Get actuators interface for drivetrain.
    std::unique_ptr<ActuatorInterface> &actuators = hardware->actuators;

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

bool SetupIntakeInterface(
    std::unique_ptr<HardwareInterface> &hardware,
    std::shared_ptr<IntakeHardwareInterface> *interface) {
    OKC_CHECK(interface != nullptr);
    OKC_CHECK(hardware->actuators != nullptr);
    OKC_CHECK(hardware->sensors != nullptr);

    // Get actuators interface for intake.
    std::unique_ptr<ActuatorInterface> &actuators = hardware->actuators;
    std::unique_ptr<SensorInterface> &sensors = hardware->sensors;


    // Set up drivetrain interface.
    IntakeHardwareInterface intake_interface = {
        actuators->intake_position_motor.get(),
        actuators->intake_motor.get(),
        actuators->indexer_motor.get(),
        
        sensors->retract_limit_switch,
        sensors->deploy_limit_switch,

        actuators->intake_position_motor.GetEncoder()
    };

    // Set the output interface
    *interface =
        std::make_shared<IntakeHardwareInterface>(intae_interface);

    return true;
}