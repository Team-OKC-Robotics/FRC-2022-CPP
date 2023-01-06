
#include "subsystems/Vision.h"

bool Vision::Init() {
    OKC_CHECK(interface_ != nullptr);

    // Update the configuration
    // TODO: parameterize the config.
    interface_->camera_config.driver_mode = false;
    interface_->camera_config.pipeline_index = 1;
    interface_->update_config = true;

    return true;
}

void Vision::Periodic() {
    VOKC_CHECK(interface_ != nullptr);

    // Update shuffleboard if not in competition mode
    bool is_competition = RobotParams::GetParam("competition", false);
    if (!is_competition) {
        // TODO: shuffleboard
    }
}

void Vision::SimulationPeriodic() {
    //
}
