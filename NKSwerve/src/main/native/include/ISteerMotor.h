#pragma once

#include <frc/geometry/Rotation2d.h>

namespace NKSwerve {

class ISteerMotor {
public:
    virtual ~ISteerMotor() = default;

    virtual void SetAngle(frc::Rotation2d target) = 0;
    virtual frc::Rotation2d GetAbsoluteRotation() const = 0;
};

}  // namespace NKSwerve
