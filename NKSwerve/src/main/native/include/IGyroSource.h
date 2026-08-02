#pragma once

#include <frc/geometry/Rotation2d.h>

namespace NKSwerve {

class IGyroSource {
public:
    virtual ~IGyroSource() = default;

    virtual frc::Rotation2d GetRotation2d() const = 0;
    virtual void Reset() = 0;
};

}  // namespace NKSwerve
