
#include "commands/drivetrain/SetSpeedDrive.h"

void SetSpeedDrive::Execute() {
    VOKC_CHECK(drivetrain_ != nullptr);

    VOKC_CALL(drivetrain_->ArcadeDrive(speed_, 0.0, false));
}
