
#pragma once

#include <memory>

#include <frc/Relay.h>
#include <frc2/command/SubsystemBase.h>
#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonUtils.h>

#include "Utils.h"

typedef struct camera_config_t {
    bool driver_mode = false;
    bool pipeline_index = 1;
} CameraConfig;

typedef struct vision_hardware_interface_t {
    // == actuators ==
    frc::Relay *const led_relay;

    // == sensors ==
    photonlib::PhotonCamera *const camera;
} VisionHardwareInterface;

typedef struct vision_software_interface_t {
    // SW INPUTS
    bool found_target;
    double pitch;
    double yaw;

    // SW OUTPUTS

    // Configure camera variables
    CameraConfig camera_config;
    bool update_config;

    // LEDs
    bool leds_on = false;
} VisionSoftwareInterface;

class VisionIO : public frc2::SubsystemBase {
public:
    VisionIO(VisionHardwareInterface *hw_interface,
             VisionSoftwareInterface *sw_interface)
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
    bool UpdateCameraConfig(CameraConfig &config);
    bool GetOrientation(bool *has_targets, double *pitch, double *yaw);

    VisionHardwareInterface *const hw_interface_;
    VisionSoftwareInterface *const sw_interface_;
};