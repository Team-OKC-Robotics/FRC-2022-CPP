
#include "io/ShooterIO.h"

void ShooterIO::Periodic() {
    // Process all the inputs and outputs to/from high level software.
    VOKC_CALL(ProcessIO());
}

void ShooterIO::SimulationPeriodic() {
    // SimulationPeriodic
}

bool ShooterIO::ProcessIO() {
    OKC_CHECK(sw_interface_ != nullptr);
    OKC_CHECK(hw_interface_ != nullptr);

    return true;
}
