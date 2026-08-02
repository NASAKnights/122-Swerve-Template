#pragma once

#include <units/length.h>
#include <units/velocity.h>

namespace NKSwerve {

class IDriveMotor {
public:
    virtual ~IDriveMotor() = default;

    virtual void SetVelocity(units::meters_per_second_t velocity) = 0;
    virtual units::meter_t GetPosition() const = 0;
    virtual units::meters_per_second_t GetVelocity() const = 0;
    virtual void ResetPosition(units::meter_t position = 0_m) = 0;
};

}  // namespace NKSwerve
