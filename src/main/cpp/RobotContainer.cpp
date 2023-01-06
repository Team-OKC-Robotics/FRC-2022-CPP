// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>

RobotContainer::RobotContainer() {
    // Load robot parameters
    VOKC_CALL(RobotParams::LoadParameters(RobotParams::param_file));

    // Initialize the hardware interface.
    hardware_ = std::make_unique<HardwareInterface>();
    VOKC_CALL(this->InitHardware(hardware_));

    VOKC_CALL(InitSubsystems());

    // TODO: put other subsystems here.

    // Initialize the Gamepads
    VOKC_CALL(InitGamepads());

    // Initialize the commands
    VOKC_CALL(InitCommands());

    // Configure the button bindings
    ConfigureButtonBindings();
}

bool RobotContainer::InitSubsystems() {
    OKC_CHECK(hardware_ != nullptr);

    // Initialize the hardware interface
    std::shared_ptr<DrivetrainHardwareInterface> drivetrain_hw;
    OKC_CALL(SetupDrivetrainInterface(hardware_, &drivetrain_hw));

    // Initialize the software interface
    drivetrain_sw_ = std::make_shared<DrivetrainSoftwareInterface>();

    // Link DrivetrainIO to hardware / software
    drivetrain_io_ = std::make_shared<DrivetrainIO>(drivetrain_hw.get(),
                                                    drivetrain_sw_.get());

    // Link Drivetrain software to the I/O
    drivetrain_ = std::make_shared<Drivetrain>(drivetrain_sw_.get());
    OKC_CALL(drivetrain_->Init());

    // == intake ==
    std::shared_ptr<IntakeHardwareInterface> intake_hw;
    OKC_CALL(SetupIntakeInterface(hardware_, &intake_hw));

    // Initialize the software interface
    intake_sw_ = std::make_shared<IntakeSoftwareInterface>();

    // Link IntakeIO to hardware / software
    intake_io_ = std::make_shared<IntakeIO>(intake_hw.get(), intake_sw_.get());

    // Link intake software to the I/O
    intake_ = std::make_shared<Intake>(intake_sw_.get());
    OKC_CALL(intake_->Init());

    // == Shooter ==
    std::shared_ptr<ShooterHardwareInterface> shooter_hw;
    OKC_CALL(SetupShooterInterface(hardware_, &shooter_hw));

    // Initialize the software interface
    shooter_sw_ = std::make_shared<ShooterSoftwareInterface>();

    // Link ShooterIO to hardware / software
    shooter_io_ =
        std::make_shared<ShooterIO>(shooter_hw.get(), shooter_sw_.get());

    // Link shooter software to the I/O
    shooter_ = std::make_shared<Shooter>(shooter_sw_.get());
    OKC_CALL(shooter_->Init());

    // == Vision ==
    std::shared_ptr<VisionHardwareInterface> vision_hw;
    OKC_CALL(SetupVisionInterface(hardware_, &vision_hw));

    // Initialize the software interface
    vision_sw_ = std::make_shared<VisionSoftwareInterface>();

    // Link the VisionIO to the hardware and software
    vision_io_ = std::make_shared<VisionIO>(vision_hw.get(), vision_sw_.get());

    // Link vision software to the I/O
    vision_ = std::make_shared<Vision>(vision_sw_.get());
    OKC_CALL(vision_->Init());

    return true;
}

void RobotContainer::ConfigureButtonBindings() {
    VOKC_CHECK(driver_a_button_ != nullptr);
    VOKC_CHECK(driver_b_button_ != nullptr);
    VOKC_CHECK(driver_back_button_ != nullptr);

    // Configure your button bindings here
    driver_back_button_->OnTrue(teleop_drive_command_.get());
    driver_a_button_->OnTrue(slow_teleop_drive_.get())
        .OnFalse(teleop_drive_command_.get());
    driver_b_button_->OnTrue(quick_teleop_drive_command_.get())
        .OnFalse(teleop_drive_command_.get());

    // Shooter
    VOKC_CHECK(manip_a_button_ != nullptr);
    VOKC_CHECK(manip_b_button_ != nullptr);
    VOKC_CHECK(driver_left_bumper_ != nullptr);
    VOKC_CHECK(driver_right_bumper_ != nullptr);

    manip_a_button_->OnTrue(shooter_preset_command_.get())
        .OnFalse(stop_shooter_command_.get());
    manip_b_button_->OnTrue(feed_command_.get())
        .OnFalse(stop_trigger_command_.get())
        .OnTrue(indexer_in_.get())
        .OnFalse(stop_indexer_.get());

    // Intake
    VOKC_CHECK(manip_back_button_ != nullptr);
    VOKC_CHECK(manip_start_button_ != nullptr);
    VOKC_CHECK(manip_left_stick_button_ != nullptr);
    VOKC_CHECK(manip_right_stick_button_ != nullptr);

    driver_left_bumper_->OnTrue(intake_in_.get()).OnFalse(stop_intake_.get());
    driver_right_bumper_->OnTrue(intake_out_.get()).OnFalse(stop_intake_.get());
    manip_back_button_->OnTrue(indexer_out_.get())
        .OnTrue(trigger_out_.get())
        .OnFalse(stop_indexer_.get())
        .OnFalse(stop_trigger_command_.get());
    manip_start_button_->WhileTrue(indexer_in_.get())
        .WhileTrue(trigger_in_.get())
        .OnFalse(stop_indexer_.get())
        .OnFalse(stop_trigger_command_.get());
    manip_left_stick_button_->OnTrue(deploy_intake_.get());
    manip_right_stick_button_->OnTrue(retract_intake_.get());
}

std::shared_ptr<frc2::Command> RobotContainer::GetAutonomousCommand() {
    // An example command will be run in autonomous
    return m_autonomousCommand;
}

std::shared_ptr<frc2::Command> RobotContainer::GetDriveCommand() {
    return teleop_drive_command_;
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

    actuators_interface->intake_position_motor =
        std::make_unique<rev::CANSparkMax>(INTAKE_POSITION_MOTOR, BRUSHLESS);
    actuators_interface->intake_motor =
        std::make_unique<rev::CANSparkMax>(INTAKE_MOTOR, BRUSHLESS);
    actuators_interface->indexer_motor =
        std::make_unique<rev::CANSparkMax>(INDEXER_MOTOR, BRUSHLESS);

    actuators_interface->intake_position_motor->SetIdleMode(COAST);
    actuators_interface->intake_motor->SetIdleMode(COAST);
    actuators_interface->indexer_motor->SetIdleMode(COAST);

    // Shooter actuators
    actuators_interface->shooter_motor =
        std::make_unique<ctre_can::TalonFX>(SHOOTER_MOTOR);
    actuators_interface->trigger_motor =
        std::make_unique<rev::CANSparkMax>(TRIGGER_MOTOR, BRUSHLESS);

    actuators_interface->trigger_motor->SetIdleMode(COAST);
    actuators_interface->trigger_motor->SetSmartCurrentLimit(30);

    // Vision actuators
    actuators_interface->led_relay =
        std::make_unique<frc::Relay>(2, frc::Relay::kBothDirections);

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

    sensor_interface->deploy_limit_switch =
        std::make_unique<frc::DigitalInput>(DEPLOY_LIMIT_SWITCH);
    sensor_interface->retracted_limit_switch =
        std::make_unique<frc::DigitalInput>(RETRACTED_LIMIT_SWITCH);

    // Shooter sensors
    sensor_interface->ball_detector =
        std::make_unique<frc::DigitalInput>(BALL_DETECTOR);

    // Initialize the camera
    sensor_interface->camera =
        std::make_unique<photonlib::PhotonCamera>("mmal_service_16.1");

    return true;
}

bool RobotContainer::InitGamepads() {
    // Get joystick IDs from parameters.toml
    int gamepad1_id = RobotParams::GetParam("gamepad1_id", 0);
    int gamepad2_id = RobotParams::GetParam("gamepad2_id", 2);

    gamepad1_ = std::make_shared<frc::Joystick>(gamepad1_id);
    gamepad2_ = std::make_shared<frc::Joystick>(gamepad2_id);

    // Initialize the joystick buttons
    driver_a_button_ =
        std::make_shared<frc2::JoystickButton>(gamepad1_.get(), A_BUTTON);
    driver_b_button_ =
        std::make_shared<frc2::JoystickButton>(gamepad1_.get(), B_BUTTON);
    driver_back_button_ =
        std::make_shared<frc2::JoystickButton>(gamepad1_.get(), BACK_BUTTON);

    // Shooter
    manip_a_button_ =
        std::make_shared<frc2::JoystickButton>(gamepad2_.get(), A_BUTTON);
    manip_b_button_ =
        std::make_shared<frc2::JoystickButton>(gamepad2_.get(), B_BUTTON);

    // Intake
    driver_left_bumper_ =
        std::make_shared<frc2::JoystickButton>(gamepad1_.get(), LEFT_BUMP);
    driver_right_bumper_ =
        std::make_shared<frc2::JoystickButton>(gamepad1_.get(), RIGHT_BUMP);
    manip_back_button_ =
        std::make_shared<frc2::JoystickButton>(gamepad2_.get(), BACK_BUTTON);
    manip_start_button_ =
        std::make_shared<frc2::JoystickButton>(gamepad2_.get(), START_BUTTON);
    manip_left_stick_button_ = std::make_shared<frc2::JoystickButton>(
        gamepad2_.get(), LEFT_STICK_BUTTON);
    manip_right_stick_button_ = std::make_shared<frc2::JoystickButton>(
        gamepad2_.get(), RIGHT_STICK_BUTTON);

    return true;
}

bool RobotContainer::InitCommands() {
    // Placeholder autonomous command.
    m_autonomousCommand = std::make_shared<ExampleCommand>();

    // Init drivetrain commands.
    quick_teleop_drive_command_ =
        std::make_shared<QuickTeleopDriveCommand>(drivetrain_, gamepad1_);
    slow_teleop_drive_ =
        std::make_shared<SlowTeleopDrive>(drivetrain_, gamepad1_);
    teleop_drive_command_ =
        std::make_shared<TeleopDriveCommand>(drivetrain_, gamepad1_);

    // Init shooter commands
    // TODO: make these parameterized.
    feed_command_ = std::make_shared<FeedCommand>(shooter_, 0.4);
    shooter_preset_command_ =
        std::make_shared<ShooterPresetCommand>(shooter_, gamepad2_, 0.4);
    stop_shooter_command_ = std::make_shared<StopShooterCommand>(shooter_);
    stop_trigger_command_ = std::make_shared<SetTriggerCommand>(shooter_, 0);
    trigger_in_ = std::make_shared<SetTriggerCommand>(shooter_, 0.4);
    trigger_out_ = std::make_shared<SetTriggerCommand>(shooter_, -0.4);

    // Init intake commands
    // TODO: make these parameterized as needed.
    indexer_in_ = std::make_shared<SetIndexerCommand>(intake_, 0.5);
    indexer_out_ = std::make_shared<SetIndexerCommand>(intake_, -0.5);
    stop_indexer_ = std::make_shared<SetIndexerCommand>(intake_, 0.0);
    intake_in_ = std::make_shared<SetIntakeCommand>(intake_, 0.8);
    intake_out_ = std::make_shared<SetIntakeCommand>(intake_, -0.5);
    stop_intake_ = std::make_shared<SetIntakeCommand>(intake_, 0.0);
    deploy_intake_ = std::make_shared<SetIntakePositionCommand>(intake_, true);
    retract_intake_ =
        std::make_shared<SetIntakePositionCommand>(intake_, false);

    return true;
}
