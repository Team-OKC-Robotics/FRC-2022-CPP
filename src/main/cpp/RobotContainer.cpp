// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>

RobotContainer::RobotContainer() {
    // Initialize the hardware interface.
    hardware_ = std::make_unique<HardwareInterface>();
    VOKC_CALL(this->InitHardware(hardware_));

    // Initialize the hardware interface
    std::shared_ptr<DrivetrainHardwareInterface> drivetrain_hw;
    VOKC_CALL(SetupDrivetrainInterface(hardware_, &drivetrain_hw));

    // Initialize the software interface
    drivetrain_sw_ = std::make_shared<DrivetrainSoftwareInterface>();

    // Link DrivetrainIO to hardware / software
    drivetrain_io_ = std::make_shared<DrivetrainIO>(drivetrain_hw.get(),
                                                    drivetrain_sw_.get());

    // Link Drivetrain software to the I/O
    drivetrain_ = std::make_shared<Drivetrain>(drivetrain_sw_.get());


    // == intake ==
    std::shared_ptr<IntakeHardwareInterface> intake_hw;
    VOKC_CALL(SetupIntakeInterface(hardware_, &intake_hw));

    // Initialize the software interface
    intake_sw_ = std::make_shared<IntakeSoftwareInterface>();

    // Link DrivetrainIO to hardware / software
    intake_io_ = std::make_shared<IntakeIO>(intake_hw.get(),
                                                    intake_sw_.get());

    // Link Drivetrain software to the I/O
    intake_ = std::make_shared<Intake>(intake_sw_.get());


    // TODO: put other subsystems here.

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

bool RobotContainer::InitHardware(
    std::unique_ptr<HardwareInterface> &hardware) {
    OKC_CHECK(hardware != nullptr);

    // Initialize sub-hardware interfaces.
    hardware->actuators = std::make_unique<ActuatorInterface>();
    hardware->sensors = std::make_unique<SensorInterface>();

    // Initialize the actuators.
    OKC_CALL(this->InitActuators(hardware->actuators.get()));

    // Set up sensors.
    OKC_CALL(this->InitSensors(*hardware->actuators, hardware->sensors.get()));

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

    actuators_interface->intake_position_motor = std::make_unique<rev::CANSparkMax>(INTAKE_POSITION_MOTOR, BRUSHLESS);
    actuators_interface->intake_motor = std::make_unique<rev::CANSparkMax>(INTAKE_MOTOR, BRUSHLESS);
    actuators_interface->indexer_motor = std::make_unique<rev::CANSparkMax>(INDEXER_MOTOR, BRUSHLESS);

    actuators_interface->intake_position_motor->SetIdleMode(COAST);
    actuators_interface->intake_motor->SetIdleMode(COAST);
    actuators_interface->indexer_motor->SetIdleMode(COAST);
    
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

    sensor_interface->deploy_limit_switch = std::make_unique<frc::DigitalInput>(DEPLOY_LIMIT_SWITCH);
    sensor_interface->retracted_limit_switch = std::make_unique<frc::DigitalInput>(RETRACTED_LIMIT_SWITCH);

    return true;
}
