// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>

RobotContainer::RobotContainer() {
    // Initialize the hardware.
    VOKC_CALL(this->InitHardware());

    // Link subsystems to hardware.
    std::shared_ptr<DrivetrainInterface> drivetrain_interface;
    VOKC_CALL(this->SetupDrivetrainInterface(&drivetrain_interface));
    // Construct Drivetrain object.
    drivetrain_ = std::make_shared<Drivetrain>(*drivetrain_interface);

    // Configure the button bindings
    ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
    // Configure your button bindings here
}

frc2::Command *RobotContainer::GetAutonomousCommand() {
    // An example command will be run in autonomous
    return &m_autonomousCommand;
}

bool RobotContainer::InitHardware() {
    // Initialize the hardware interface.
    hardware_ = std::make_unique<HardwareInterface>();

    // Initialize sub-hardware interfaces.
    hardware_->actuators = std::make_unique<ActuatorInterface>();
    hardware_->sensors = std::make_unique<SensorInterface>();

    // Initialize the actuators.
    OKC_CALL(this->InitActuators(hardware_->actuators.get()));

    // Set up sensors.
    OKC_CALL(
        this->InitSensors(*hardware_->actuators, hardware_->sensors.get()));

    return true;
}

bool RobotContainer::InitActuators(ActuatorInterface *actuators_interface) {
    OKC_CHECK(actuators_interface != nullptr);

    // Initialize drivetrain motors.
    actuators_interface->left_motor_1 =
        std::make_unique<rev::CANSparkMax>(LEFT_MOTOR_1, BRUSHLESS);
    actuators_interface->left_motor_2 =
        std::make_unique<rev::CANSparkMax>(LEFT_MOTOR_2, BRUSHLESS);
    actuators_interface->left_motor_3 =
        std::make_unique<rev::CANSparkMax>(LEFT_MOTOR_3, BRUSHLESS);
    actuators_interface->right_motor_1 =
        std::make_unique<rev::CANSparkMax>(RIGHT_MOTOR_1, BRUSHLESS);
    actuators_interface->right_motor_2 =
        std::make_unique<rev::CANSparkMax>(RIGHT_MOTOR_2, BRUSHLESS);
    actuators_interface->right_motor_3 =
        std::make_unique<rev::CANSparkMax>(RIGHT_MOTOR_3, BRUSHLESS);

    // Set motors to coast to protect the gearbox and motors. This also makes
    // driving easier
    actuators_interface->left_motor_1->SetIdleMode(COAST);
    actuators_interface->left_motor_2->SetIdleMode(COAST);
    actuators_interface->left_motor_3->SetIdleMode(COAST);
    actuators_interface->right_motor_1->SetIdleMode(COAST);
    actuators_interface->right_motor_2->SetIdleMode(COAST);
    actuators_interface->right_motor_3->SetIdleMode(COAST);

    return true;
}

bool RobotContainer::InitSensors(const ActuatorInterface &actuators,
                                 SensorInterface *sensor_interface) {
    OKC_CHECK(sensor_interface != nullptr);

    // Initialize navX.
    try {
        sensor_interface->ahrs = std::make_unique<AHRS>(frc::SPI::Port::kMXP);
    } catch (std::exception &ex) {
        std::string what_string = ex.what();
        std::string err_msg("Error instantiating navX MXP:  " + what_string);
        const char *p_err_msg = err_msg.c_str();

        // Print the error message.
        OKC_CHECK_MSG(false, p_err_msg);
    }

    return true;
}

bool RobotContainer::SetupDrivetrainInterface(
    std::shared_ptr<DrivetrainInterface> *interface) {
    OKC_CHECK(interface != nullptr);
    OKC_CHECK(hardware_->actuators != nullptr);
    OKC_CHECK(hardware_->sensors != nullptr);

    // Get actuators interface for drivetrain.
    std::unique_ptr<ActuatorInterface> &actuators = hardware_->actuators;

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
    OKC_CHECK(hardware_->diff_drive == nullptr);

    // Create the differential drive.
    hardware_->diff_drive =
        std::make_unique<frc::DifferentialDrive>(left_motors, right_motors);

    // Set up drivetrain interface.
    DrivetrainInterface drivetrain_interface = {
        actuators->left_motor_1.get(),  actuators->left_motor_2.get(),
        actuators->left_motor_3.get(),  actuators->right_motor_1.get(),
        actuators->right_motor_2.get(), actuators->right_motor_3.get(),
        hardware_->diff_drive.get(),    hardware_->sensors->ahrs.get()};

    // Set the output interface
    *interface = std::make_shared<DrivetrainInterface>(drivetrain_interface);

    return true;
}
