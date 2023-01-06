
#include "io/VisionIO.h"

void VisionIO::Periodic() {
    // Process all the inputs and outputs to/from high level software.
    VOKC_CALL(ProcessIO());
}

void VisionIO::SimulationPeriodic() {
    // SimulationPeriodic
}

bool VisionIO::ProcessIO() {
    OKC_CHECK(sw_interface_ != nullptr);
    OKC_CHECK(hw_interface_ != nullptr);

    // Change LED settings.

    // Configure the camera as needed.
    if (sw_interface_->update_config) {
        OKC_CALL(UpdateCameraConfig(sw_interface_->camera_config));
        sw_interface_->update_config = false;
    }

    // Detect targets and find the orientation with respect to the best target.
    OKC_CALL(GetOrientation(&sw_interface_->found_target, &sw_interface_->pitch,
                            &sw_interface_->yaw));

    return true;
}

bool VisionIO::GetOrientation(bool *has_targets, double *pitch, double *yaw) {
    OKC_CHECK(hw_interface_->camera != nullptr);
    OKC_CHECK(pitch != nullptr);
    OKC_CHECK(yaw != nullptr);
    OKC_CHECK(has_targets != nullptr);

    photonlib::PhotonPipelineResult result =
        hw_interface_->camera->GetLatestResult();

    *has_targets = result.HasTargets();
    if (*has_targets) {
        photonlib::PhotonTrackedTarget best_target = result.GetBestTarget();
        *pitch = best_target.GetPitch();
        *yaw = best_target.GetYaw();
    } else {
        // Otherwise, no target was found. Don't update the pitch and yaw.
    }

    return true;
}

bool VisionIO::UpdateCameraConfig(CameraConfig &config) {
    OKC_CHECK(sw_interface_ != nullptr);
    OKC_CHECK(hw_interface_ != nullptr);
    OKC_CHECK(hw_interface_->camera != nullptr);

    // Update the camera configuration
    hw_interface_->camera->SetDriverMode(config.driver_mode);
    hw_interface_->camera->SetPipelineIndex(config.pipeline_index);

    return true;
}