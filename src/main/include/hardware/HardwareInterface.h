
#pragma once

#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/MotorControllerGroup.h>

#include "hardware/ActuatorInterface.h"
#include "hardware/SensorInterface.h"

// Subsystems
#include "subsystems/Drivetrain.h"

typedef struct hardware_t {
    // Actuators
    std::unique_ptr<ActuatorInterface> actuators;

    // Drivetrain specific hardware abstractions.
    std::unique_ptr<frc::DifferentialDrive> diff_drive;

    // Sensors
    std::unique_ptr<SensorInterface> sensors;

} HardwareInterface;

// Subsystem hardware setup functions

/**
 * @brief Link the Drivetrain to the hardware interfaces.
 *
 * @param interface
 * @return true
 * @return false
 */
bool SetupDrivetrainInterface(std::unique_ptr<HardwareInterface> &hardware,
                              std::shared_ptr<DrivetrainInterface> *interface);
