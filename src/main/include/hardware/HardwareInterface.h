
#pragma once

#include <frc/drive/DifferentialDrive.h>

#include "hardware/ActuatorInterface.h"
#include "hardware/SensorInterface.h"

typedef struct hardware_t {
    // Actuators
    std::unique_ptr<ActuatorInterface> actuators;

    // Drivetrain specific hardware abstractions.
    std::unique_ptr<frc::DifferentialDrive> diff_drive;

    // Sensors
    std::unique_ptr<SensorInterface> sensors;

} HardwareInterface;
