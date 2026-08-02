#pragma once

#include <memory>

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>

#include "IDriveMotor.h"
#include "ISteerMotor.h"

namespace NKSwerve {

// Plain class, not a frc2::SubsystemBase — owned and driven directly by
// SwerveDrive, with no thread function of its own.
class SwerveModule {
public:
    SwerveModule(std::unique_ptr<IDriveMotor> driveMotor,
                 std::unique_ptr<ISteerMotor> steerMotor);

    frc::SwerveModuleState GetCurrentState() const;
    frc::SwerveModulePosition GetPosition() const;
    void SetDesiredState(frc::SwerveModuleState desiredState);
    void ResetDriveEncoder();
    frc::Rotation2d GetAbsoluteRotation() const;

private:
    std::unique_ptr<IDriveMotor> m_driveMotor;
    std::unique_ptr<ISteerMotor> m_steerMotor;
};

}  // namespace NKSwerve
