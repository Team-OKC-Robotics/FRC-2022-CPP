#pragma once

#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>

class SwerveModule {
public:
    bool SetState(frc::SwerveModuleState &state);
    bool GetState(frc::SwerveModuleState *state);
    bool GetPosition(frc::SwerveModulePosition *pos);
private:
    frc::SwerveModuleState state;
    frc::SwerveModulePosition pos;
};