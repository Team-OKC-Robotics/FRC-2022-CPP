
#pragma once

#include <memory>

#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/DigitalInput.h>
#include <rev/CANSparkMax.h>

#include "Utils.h"


typedef struct intake_config_t {
    double max_output_retract;
    double max_output_deploy;
    double open_loop_ramp_rate;
    double max_indexer_current;
    double EXTENDED;
    double RETRACTED;
} IntakeConfig;

typedef struct intake_hardware_interface_t {
    // == actuators ==

    // Intake position motor
    rev::CANSparkMax *const intake_position_motor;

    // intake intake motor (the one that actually sucks the cargo in)
    rev::CANSparkMax *const intake_motor;

    // indexer motor (the one with the polycord, be careful, it's a Neo 550)
    rev::CANSparkMax *const indexer_motor;


    // == sensors ==
    frc::DigitalInput *const retract_limit_switch;
    frc::DigitalInput *const deploy_limit_switch;

    rev::RelativeEncoder *const intake_position_encoder;
} IntakeHardwareInterface;

typedef struct intake_software_interface_t {
    // SW INPUTS

    // limit switches
    bool deployed_limit_switch_val;
    bool retracted_limit_switch_val;

    // Encoders
    double intake_position_encoder_val;




    // SW OUTPUTS

    // Configure intake variables
    IntakeConfig intake_config;
    bool update_config;

    // Reset flags
    bool reset_encoders;

    bool set_encoder_to_val;
    double encoder_val_to_set;

    // actuator outputs
    double intake_position_power;
    double intake_power;
    double indexer_power;
} IntakeSoftwareInterface;

class IntakeIO : public frc2::SubsystemBase {
public:
    IntakeIO(IntakeHardwareInterface *hw_interface,
                 IntakeSoftwareInterface *sw_interface)
        : hw_interface_(hw_interface), sw_interface_(sw_interface) {}

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;

    /**
     * Will be called periodically whenever the CommandScheduler runs during
     * simulation.
     */
    void SimulationPeriodic() override;

    bool ProcessIO();

private:
    bool UpdateIntakeConfig(IntakeConfig &config);
    bool ResetEncoders();
    bool SetEncoder(double &val);

    IntakeHardwareInterface *const hw_interface_;
    IntakeSoftwareInterface *const sw_interface_;
};