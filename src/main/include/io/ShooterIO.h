
#pragma once

#include <memory>

#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#incld
#include <frc/DigitalInput.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

#include "Utils.h"

namespace ctre_mc = ctre::phoenix::motorcontrol;
namespace ctre_can = ctre::phoenix::motorcontrol::can;

typedef struct shooter_motor_config {
    bool inverted;
    double neutral_deadband;
    ctre_mc::NeutralMode neutral_mode;
    double Kp;
    double Ki;
    double Kd;
    double open_loop_ramp_rate;
    double sensor_pos;
} ShooterMotorConfig;

typedef struct trigger_motor_config {
    double smart_current_limit;
} TriggerMotorConfig;

typedef struct shooter_hardware_interface_t {
    // == actuators ==

    // Shooter motor
    ctre_can::TalonFX *const shooter_motor;

    // Trigger motor
    rev::CANSparkMax *const trigger_motor;

    // == sensors ==
    frc::DigitalInput *const ball_detector;

} ShooterHardwareInterface;

typedef struct shooter_software_interface_t {
    // SW INPUTS
    bool is_ball_detected;

    double shooter_rpm;
    double shooter_output_pct;
    double ticks;
    double velocity_error;

    // SW OUTPUTS
    bool stop_shooter;
    double shooter_power;

    double trigger_power;

    ShooterMotorConfig shooter_config;
    TriggerMotorConfig trigger_config;

    bool update_shooter_config;
    bool update_trigger_config;

} ShooterSoftwareInterface;

class ShooterIO : public frc2::SubsystemBase {
public:
    ShooterIO(ShooterHardwareInterface *hw_interface,
              ShooterSoftwareInterface *sw_interface)
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
    bool UpdateShooterConfig();
    bool UpdateTriggerConfig();
    bool SetMotorOutputs();
    bool ReadSensors();

    ShooterHardwareInterface *const hw_interface_;
    ShooterSoftwareInterface *const sw_interface_;
};