#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>

RobotContainer::RobotContainer() {
    // Load robot parameters
    VOKC_CALL(RobotParams::LoadParameters(RobotParams::param_file));

    // Initialize the hardware interface.
    hardware_ = std::make_unique<HardwareInterface>();
    VOKC_CALL(this->InitHardware(hardware_));

    // == swerve drive ==
    VOKC_CALL(SetupSwerveDriveInterface(hardware_, swerve_drive_hw));

    // Initialize the software interface
    swerve_drive_sw_ = std::make_shared<SwerveDriveSoftwareInterface>();

    // Link SwerveDriveIO to hardware / software
    swerve_drive_io_ = std::make_shared<SwerveDriveIO>(swerve_drive_hw.get(), swerve_drive_sw_.get());

    // Link swerve dirve software to the I/O
    swerve_drive_ = std::make_shared<SwerveDrive>(swerve_drive_sw_.get());

    VOKC_CHECK(swerve_drive_hw != nullptr);
    VOKC_CHECK(swerve_drive_sw_ != nullptr);
    VOKC_CHECK(swerve_drive_io_ != nullptr);
    VOKC_CHECK(swerve_drive_ != nullptr);
    
    VOKC_CALL(swerve_drive_->Init());

    // Initialize the Gamepads
    VOKC_CALL(InitGamepads());

    // Initialize the commands
    VOKC_CALL(InitCommands());

    // Configure the button bindings
    ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
    VOKC_CHECK(driver_a_button_ != nullptr);
    VOKC_CHECK(driver_b_button_ != nullptr);
    VOKC_CHECK(driver_back_button_ != nullptr);

    // Configure your button bindings here
    // driver_back_button_->OnTrue(teleop_drive_command_.get());
    // driver_a_button_->OnTrue(slow_teleop_drive_.get())
    //     .OnFalse(teleop_drive_command_.get());
    // driver_b_button_->OnTrue(quick_teleop_drive_command_.get())
    //     .OnFalse(teleop_drive_command_.get());

    VOKC_CHECK(manip_a_button_ != nullptr);
    VOKC_CHECK(manip_b_button_ != nullptr);
    VOKC_CHECK(driver_left_bumper_ != nullptr);
    VOKC_CHECK(driver_right_bumper_ != nullptr);

    VOKC_CHECK(manip_back_button_ != nullptr);
    VOKC_CHECK(manip_start_button_ != nullptr);
    VOKC_CHECK(manip_left_stick_button_ != nullptr);
    VOKC_CHECK(manip_right_stick_button_ != nullptr);
}

std::shared_ptr<frc2::Command> RobotContainer::GetAutonomousCommand() {
    // An example command will be run in autonomous
    return m_autonomousCommand;
}

std::shared_ptr<frc2::Command> RobotContainer::GetDriveCommand() {
    return swerve_teleop_command_;
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

    actuators_interface->left_front_drive_motor = std::make_unique<rev::CANSparkMax>(LEFT_FRONT_DRIVE_MOTOR, BRUSHLESS);
    actuators_interface->left_back_drive_motor = std::make_unique<rev::CANSparkMax>(LEFT_BACK_DRIVE_MOTOR, BRUSHLESS);
    actuators_interface->right_front_drive_motor = std::make_unique<rev::CANSparkMax>(RIGHT_FRONT_DRIVE_MOTOR, BRUSHLESS);
    actuators_interface->right_back_drive_motor = std::make_unique<rev::CANSparkMax>(RIGHT_BACK_DRIVE_MOTOR, BRUSHLESS);

    actuators_interface->left_front_steer_motor = std::make_unique<rev::CANSparkMax>(LEFT_FRONT_STEER_MOTOR, BRUSHLESS);
    actuators_interface->left_back_steer_motor = std::make_unique<rev::CANSparkMax>(LEFT_BACK_STEER_MOTOR, BRUSHLESS);
    actuators_interface->right_front_steer_motor = std::make_unique<rev::CANSparkMax>(RIGHT_FRONT_STEER_MOTOR, BRUSHLESS);
    actuators_interface->right_back_steer_motor = std::make_unique<rev::CANSparkMax>(RIGHT_BACK_STEER_MOTOR, BRUSHLESS);

    OKC_CHECK(actuators_interface->left_front_drive_motor != nullptr);
    OKC_CHECK(actuators_interface->left_back_drive_motor != nullptr);
    OKC_CHECK(actuators_interface->right_front_drive_motor != nullptr);
    OKC_CHECK(actuators_interface->right_back_drive_motor != nullptr);

    OKC_CHECK(actuators_interface->left_front_steer_motor != nullptr);
    OKC_CHECK(actuators_interface->left_back_steer_motor != nullptr);
    OKC_CHECK(actuators_interface->right_front_steer_motor != nullptr);
    OKC_CHECK(actuators_interface->right_back_steer_motor != nullptr);

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

    OKC_CHECK(actuators.left_front_drive_motor != nullptr);
    OKC_CHECK(actuators.left_back_drive_motor != nullptr);
    OKC_CHECK(actuators.right_front_drive_motor != nullptr);
    OKC_CHECK(actuators.right_back_drive_motor != nullptr);

    OKC_CHECK(actuators.left_front_steer_motor != nullptr);
    OKC_CHECK(actuators.left_back_steer_motor != nullptr);
    OKC_CHECK(actuators.right_front_steer_motor != nullptr);
    OKC_CHECK(actuators.right_back_steer_motor != nullptr);

    sensor_interface->left_front_drive_encoder = std::make_unique<rev::SparkMaxRelativeEncoder>(actuators.left_front_drive_motor->GetEncoder());
    sensor_interface->left_back_drive_encoder = std::make_unique<rev::SparkMaxRelativeEncoder>(actuators.left_back_drive_motor->GetEncoder());
    sensor_interface->right_front_drive_encoder = std::make_unique<rev::SparkMaxRelativeEncoder>(actuators.right_front_drive_motor->GetEncoder());
    sensor_interface->right_back_drive_encoder = std::make_unique<rev::SparkMaxRelativeEncoder>(actuators.right_back_drive_motor->GetEncoder());
    
    sensor_interface->left_front_steer_encoder = std::make_unique<frc::AnalogInput>(LEFT_FRONT_STEER_ENCODER);
    sensor_interface->left_back_steer_encoder = std::make_unique<frc::AnalogInput>(LEFT_BACK_STEER_ENCODER);
    sensor_interface->right_front_steer_encoder = std::make_unique<frc::AnalogInput>(RIGHT_FRONT_STEER_ENCODER);
    sensor_interface->right_back_steer_encoder = std::make_unique<frc::AnalogInput>(RIGHT_BACK_STEER_ENCODER);

    sensor_interface->left_front_steer_vel_encoder = std::make_unique<rev::SparkMaxRelativeEncoder>(actuators.left_front_steer_motor->GetEncoder());
    sensor_interface->left_back_steer_vel_encoder = std::make_unique<rev::SparkMaxRelativeEncoder>(actuators.left_back_steer_motor->GetEncoder());
    sensor_interface->right_front_steer_vel_encoder = std::make_unique<rev::SparkMaxRelativeEncoder>(actuators.right_front_steer_motor->GetEncoder());
    sensor_interface->right_back_steer_vel_encoder = std::make_unique<rev::SparkMaxRelativeEncoder>(actuators.right_back_steer_motor->GetEncoder());

    OKC_CHECK(sensor_interface->left_front_drive_encoder != nullptr);
    OKC_CHECK(sensor_interface->left_back_drive_encoder != nullptr);
    OKC_CHECK(sensor_interface->right_front_drive_encoder != nullptr);
    OKC_CHECK(sensor_interface->right_back_drive_encoder != nullptr);

    OKC_CHECK(sensor_interface->left_front_steer_encoder != nullptr);
    OKC_CHECK(sensor_interface->left_back_steer_encoder != nullptr);
    OKC_CHECK(sensor_interface->right_front_steer_encoder != nullptr);
    OKC_CHECK(sensor_interface->right_back_steer_encoder != nullptr);

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

    manip_a_button_ =
        std::make_shared<frc2::JoystickButton>(gamepad2_.get(), A_BUTTON);
    manip_b_button_ =
        std::make_shared<frc2::JoystickButton>(gamepad2_.get(), B_BUTTON);

    return true;
}

bool RobotContainer::InitCommands() {
    OKC_CHECK(swerve_drive_ != nullptr);

    // Placeholder autonomous command.
    //m_autonomousCommand = std::make_shared<AutoSwerveCommand>(swerve_drive_.get(), frc::Pose2d());
    m_autonomousCommand = nullptr;

    swerve_teleop_command_ = std::make_shared<TeleOpSwerveCommand>(swerve_drive_, gamepad1_);
    OKC_CHECK(swerve_teleop_command_ != nullptr);

    return true;
}
