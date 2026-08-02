#pragma once

#include <array>
#include <memory>

#include <frc/geometry/Translation2d.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

#include "IDriveMotor.h"
#include "ISteerMotor.h"

namespace NKSwerve {

struct SwerveModuleHardware {
    std::unique_ptr<IDriveMotor> driveMotor;
    std::unique_ptr<ISteerMotor> steerMotor;
    frc::Translation2d position;  // module location relative to robot center, for kinematics
};

struct SwerveDriveConfig {
    std::array<SwerveModuleHardware, 4> modules;  // FL, FR, BL, BR
    units::meters_per_second_t maxTranslationalVelocity;
    units::radians_per_second_t maxRotationalVelocity;
};

}  // namespace NKSwerve
