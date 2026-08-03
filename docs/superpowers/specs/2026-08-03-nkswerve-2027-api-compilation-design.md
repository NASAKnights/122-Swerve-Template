# NKSwerve 2027 Alpha API Compilation — Design Spec

## Context

The [build scaffolding spec](2026-08-03-nkswerve-build-scaffolding-migration-design.md) got `NKSwerve/` into a real, standalone Gradle vendor-library project, but deliberately left `compileCpp` failing — `SwerveDrive.cpp`/`SwerveModule.cpp` still `#include <frc/geometry/...h>` and use `frc::ChassisSpeeds`/`frc::SwerveModuleState`/`wpi::array`, none of which exist in the installed 2027 alpha-6 WPILib. That spec's "Open items" flagged the mechanical rename as future work and explicitly deferred one real design question: whether `NKSwerve`'s public API should carry the new `*Accelerations`/`*Acceleration` types at all, or ignore them.

This spec resolves that question (thread them through as stubs — confirmed with the user) and rewrites all nine `NKSwerve/src/main/native/{cpp,include}/*` files against the **actual, verified** 2027 alpha-6 C++ API — extracted directly from the installed headers at `C:\Users\Public\wpilib\2027_alpha5\maven\org\wpilib\{wpimath,wpiutil}\...-headers.zip` (not guessed from the old `frc::` API by pattern-matching). The real API turned out to differ from a simple namespace rename in several load-bearing ways, listed below.

## Goals

- Every file in `NKSwerve/src/main/native/{cpp,include}/` compiles against the real 2027 alpha-6 headers.
- **Pass condition:** `./gradlew NKSwerveWindowsx86-64DebugStaticLibrary` (and `...ReleaseStaticLibrary`) succeeds from `NKSwerve/` — i.e., the actual `.lib` archive file builds. This is the artifact a consuming project would import.
- `wpi::math::ChassisAccelerations`/`SwerveModuleAcceleration` are threaded through the public API (`SwerveDrive::Drive`, `SwerveModule::SetDesiredState`) as confirmed with the user, with stub handling — accepted, stored where there's a natural place to store them, never computed or consumed.
- No behavior changes beyond what's mechanically required to compile. `SwerveDrive::ThreadFunction()` stays the existing no-op stub.

## Non-goals

- **No test suite work.** `NKSwerveTest`/`PlaceholderTest.cpp` are untouched — this spec's pass condition is the static library only. Real tests come later, once logic is imported from `archive/src/`.
- The `/std:c++23preview` MSVC toolchain flag issue from the build-scaffolding spec — that only affects `NKSwerveTest` (which isn't in scope here), so it's untouched.
- `SwerveDrive::ThreadFunction()`'s functional behavior (kinematics → per-module `SetDesiredState`, odometry update) — still deferred, same as every prior spec in this chain.
- Real acceleration *sensing* or *feedforward* — `GetCurrentAcceleration()`/`GetRobotRelativeAccelerations()` are stubs returning zeroed values; wiring real motor-controller acceleration feedforward is a hardware-specific decision left for whichever spec picks concrete `IDriveMotor`/`ISteerMotor` implementations (the testbed spec).
- `IDriveMotor`/`ISteerMotor`/`IGyroSource`/`IMutex` — the motor/gyro-level interfaces are **not restructured** to add acceleration parameters. Only the chassis/module *state* types they feed into (`SwerveModuleVelocity`/`SwerveModuleAcceleration`, `ChassisVelocities`/`ChassisAccelerations`) come from the renamed WPILib API; the hardware interfaces keep today's shape (`SetVelocity`, `GetPosition`, etc.), just with renamed unit types.

## Verified 2027 alpha-6 API differences

Confirmed by reading the actual headers, not inferred:

| Old (`frc::`) | New (`wpi::math::`) | Note |
|---|---|---|
| `frc::Rotation2d`, `Pose2d`, `Translation2d` | `wpi::math::Rotation2d`, `Pose2d`, `Translation2d` | Straight rename, same shape. Headers: `wpi/math/geometry/*.hpp`. |
| `frc::ChassisSpeeds` | `wpi::math::ChassisVelocities` + `wpi::math::ChassisAccelerations` | Split into two structs (`{vx, vy, omega}` / `{ax, ay, alpha}`). Headers: `wpi/math/kinematics/ChassisVelocities.hpp` / `ChassisAccelerations.hpp`. |
| `frc::SwerveModuleState` | `wpi::math::SwerveModuleVelocity` + `wpi::math::SwerveModuleAcceleration` | Split into two structs (`{velocity, angle}` / `{acceleration, angle}`). Headers: `wpi/math/kinematics/SwerveModuleVelocity.hpp` / `SwerveModuleAcceleration.hpp`. |
| `frc::SwerveModuleState::Optimize(state, currentAngle)` (**static**) | `SwerveModuleVelocity::Optimize(currentAngle)` (**instance method**) | Called as `desiredVelocity.Optimize(currentAngle)`, not a static call. Non-`const`-qualified in the header, but returns a new value rather than mutating — call on a non-const lvalue. |
| `frc::SwerveDriveKinematics<N>::ToSwerveModuleStates(...)` | `wpi::math::SwerveDriveKinematics<N>::ToSwerveModuleVelocities(...)` | Renamed. New sibling `ToSwerveModuleAccelerations(chassisAccelerations, angularVelocity = 0_rad_per_s, centerOfRotation = {})`. |
| `frc::SwerveDriveKinematics<N>::ToChassisSpeeds(...)` | `wpi::math::SwerveDriveKinematics<N>::ToChassisVelocities(...)` | Renamed. New sibling `ToChassisAccelerations(...)`. |
| `frc::SwerveDriveOdometry<N>` | `wpi::math::SwerveDriveOdometry<N>` | Same constructor shape `(kinematics, gyroAngle, modulePositions, initialPose = Pose2d{})`. Its base class (`wpi::math::Odometry<...>`) keeps the **same method names**: `ResetPosition`, `ResetPose`, `GetPose`, `Update` — only the types they take changed. No call-site renames needed here beyond types. |
| `wpi::array<T, N>` | `wpi::util::array<T, N>` | Header `wpi/util/array.hpp`. Same brace-init construction syntax (`wpi::util::array<T,4>{a,b,c,d}`) — drop-in besides the namespace. |
| `units::meters_per_second_t`, `units::meter_t`, etc. | `wpi::units::meters_per_second_t`, `wpi::units::meter_t`, etc. | Headers move from `units/*.h` to `wpi/units/*.hpp`. Literal suffixes (`0_m`, `0_mps`, `0_rad_per_s`, `0_mps_sq`, `0_rad_per_s_sq`) remain globally-scoped (confirmed by their use, unqualified, inside official WPILib headers) — usable unqualified from `NKSwerve`'s own namespace too, same as they were with the old `units::` library. |

`NKSwerve`'s own code is **not** nested inside `namespace wpi { ... }`, so — unlike WPILib's own headers, which benefit from enclosing-namespace lookup — every `wpi::math::`/`wpi::units::` **type** name must be fully qualified in `NKSwerve`'s source. Only the numeric literal suffixes are exempt (they're globally scoped).

## File-by-file changes

### `IGyroSource.h` — rename only

```cpp
#pragma once

#include "wpi/math/geometry/Rotation2d.hpp"

namespace NKSwerve {

class IGyroSource {
public:
    virtual ~IGyroSource() = default;

    virtual wpi::math::Rotation2d GetRotation2d() const = 0;
    virtual void Reset() = 0;
};

}  // namespace NKSwerve
```

### `IDriveMotor.h` — rename only

```cpp
#pragma once

#include "wpi/units/length.hpp"
#include "wpi/units/velocity.hpp"

namespace NKSwerve {

class IDriveMotor {
public:
    virtual ~IDriveMotor() = default;

    virtual void SetVelocity(wpi::units::meters_per_second_t velocity) = 0;
    virtual wpi::units::meter_t GetPosition() const = 0;
    virtual wpi::units::meters_per_second_t GetVelocity() const = 0;
    virtual void ResetPosition(wpi::units::meter_t position = 0_m) = 0;
};

}  // namespace NKSwerve
```

### `ISteerMotor.h` — rename only

```cpp
#pragma once

#include "wpi/math/geometry/Rotation2d.hpp"

namespace NKSwerve {

class ISteerMotor {
public:
    virtual ~ISteerMotor() = default;

    virtual void SetAngle(wpi::math::Rotation2d target) = 0;
    virtual wpi::math::Rotation2d GetAbsoluteRotation() const = 0;
};

}  // namespace NKSwerve
```

### `IMutex.h` — no changes

No WPILib types anywhere in this file; untouched by this spec.

### `SwerveModule.h`

```cpp
#pragma once

#include <memory>

#include "wpi/math/geometry/Rotation2d.hpp"
#include "wpi/math/kinematics/SwerveModuleAcceleration.hpp"
#include "wpi/math/kinematics/SwerveModulePosition.hpp"
#include "wpi/math/kinematics/SwerveModuleVelocity.hpp"

#include "IDriveMotor.h"
#include "ISteerMotor.h"

namespace NKSwerve {

// Plain class, not a frc2::SubsystemBase -- owned and driven directly by
// SwerveDrive, with no thread function of its own.
class SwerveModule {
public:
    SwerveModule(std::unique_ptr<IDriveMotor> driveMotor,
                 std::unique_ptr<ISteerMotor> steerMotor);

    wpi::math::SwerveModuleVelocity GetCurrentVelocity() const;

    // Stub: no acceleration sensing exists yet, always returns a
    // zeroed SwerveModuleAcceleration. Deferred to a future pass.
    wpi::math::SwerveModuleAcceleration GetCurrentAcceleration() const;

    wpi::math::SwerveModulePosition GetPosition() const;

    // desiredAcceleration is accepted but unused (stub) -- nothing wires
    // it to IDriveMotor/ISteerMotor yet. Threaded through now so the
    // signature matches the 2027 API split without redesigning
    // motor-level feedforward here.
    void SetDesiredState(wpi::math::SwerveModuleVelocity desiredVelocity,
                          wpi::math::SwerveModuleAcceleration desiredAcceleration);

    void ResetDriveEncoder();
    wpi::math::Rotation2d GetAbsoluteRotation() const;

private:
    std::unique_ptr<IDriveMotor> m_driveMotor;
    std::unique_ptr<ISteerMotor> m_steerMotor;
};

}  // namespace NKSwerve
```

### `SwerveModule.cpp`

```cpp
#include "SwerveModule.h"

#include <utility>

using namespace NKSwerve;

SwerveModule::SwerveModule(std::unique_ptr<IDriveMotor> driveMotor,
                           std::unique_ptr<ISteerMotor> steerMotor)
    : m_driveMotor(std::move(driveMotor)), m_steerMotor(std::move(steerMotor)) {}

wpi::math::SwerveModuleVelocity SwerveModule::GetCurrentVelocity() const {
    return {m_driveMotor->GetVelocity(), m_steerMotor->GetAbsoluteRotation()};
}

wpi::math::SwerveModuleAcceleration SwerveModule::GetCurrentAcceleration() const {
    return {};
}

wpi::math::SwerveModulePosition SwerveModule::GetPosition() const {
    return {m_driveMotor->GetPosition(), m_steerMotor->GetAbsoluteRotation()};
}

void SwerveModule::SetDesiredState(
    wpi::math::SwerveModuleVelocity desiredVelocity,
    [[maybe_unused]] wpi::math::SwerveModuleAcceleration desiredAcceleration) {
    wpi::math::Rotation2d currentAngle = m_steerMotor->GetAbsoluteRotation();
    desiredVelocity = desiredVelocity.Optimize(currentAngle);

    m_steerMotor->SetAngle(desiredVelocity.angle);
    m_driveMotor->SetVelocity(desiredVelocity.velocity);
}

void SwerveModule::ResetDriveEncoder() {
    m_driveMotor->ResetPosition();
}

wpi::math::Rotation2d SwerveModule::GetAbsoluteRotation() const {
    return m_steerMotor->GetAbsoluteRotation();
}
```

### `SwerveDriveConfig.h`

```cpp
#pragma once

#include <array>
#include <memory>

#include "wpi/math/geometry/Translation2d.hpp"
#include "wpi/units/angular_velocity.hpp"
#include "wpi/units/velocity.hpp"

#include "IDriveMotor.h"
#include "ISteerMotor.h"

namespace NKSwerve {

struct SwerveModuleHardware {
    std::unique_ptr<IDriveMotor> driveMotor;
    std::unique_ptr<ISteerMotor> steerMotor;
    wpi::math::Translation2d position;  // module location relative to robot center, for kinematics
};

struct SwerveDriveConfig {
    std::array<SwerveModuleHardware, 4> modules;  // FL, FR, BL, BR
    wpi::units::meters_per_second_t maxTranslationalVelocity;
    wpi::units::radians_per_second_t maxRotationalVelocity;
};

}  // namespace NKSwerve
```

### `SwerveDrive.h`

```cpp
#pragma once

#include <array>
#include <memory>

#include "wpi/math/geometry/Pose2d.hpp"
#include "wpi/math/geometry/Rotation2d.hpp"
#include "wpi/math/kinematics/ChassisAccelerations.hpp"
#include "wpi/math/kinematics/ChassisVelocities.hpp"
#include "wpi/math/kinematics/SwerveDriveKinematics.hpp"
#include "wpi/math/kinematics/SwerveDriveOdometry.hpp"
#include "wpi/math/kinematics/SwerveModulePosition.hpp"

#include "IGyroSource.h"
#include "IMutex.h"
#include "SwerveDriveConfig.h"
#include "SwerveModule.h"

namespace NKSwerve {

// Plain class, not a frc2::SubsystemBase.
class SwerveDrive {
public:
    SwerveDrive(SwerveDriveConfig config, std::shared_ptr<IGyroSource> gyro,
                std::shared_ptr<IMutex> commandMutex);

    // The one function meant to be called repeatedly by the consuming
    // application (e.g. wired into its own frc2::SubsystemBase::Periodic()).
    // Still a stub -- what it does with the posted velocities/accelerations
    // once read is deferred to a later pass.
    void ThreadFunction();

    // desiredAccelerations is accepted and stored but never consumed --
    // ThreadFunction() is still a stub. Threaded through now so the
    // signature matches the 2027 API split.
    void Drive(wpi::math::ChassisVelocities desiredVelocities,
               wpi::math::ChassisAccelerations desiredAccelerations);

    wpi::math::Rotation2d GetHeading() const;
    void ResetHeading();

    void ResetPose(wpi::math::Pose2d pose);
    wpi::math::Pose2d GetPose() const;

    std::array<wpi::math::SwerveModulePosition, 4> GetModulePositions() const;
    wpi::math::ChassisVelocities GetRobotRelativeVelocities() const;

    // Stub: no acceleration sensing exists yet, always returns a
    // zeroed ChassisAccelerations. Deferred to a future pass.
    wpi::math::ChassisAccelerations GetRobotRelativeAccelerations() const;

private:
    SwerveDriveConfig m_config;
    std::shared_ptr<IGyroSource> m_gyro;
    std::array<SwerveModule, 4> m_modules;
    wpi::math::SwerveDriveKinematics<4> m_kinematics;
    wpi::math::SwerveDriveOdometry<4> m_odometry;

    std::shared_ptr<IMutex> m_commandMutex;
    wpi::math::ChassisVelocities m_desiredVelocities;         // the notice board: latest posted Drive() velocities
    wpi::math::ChassisAccelerations m_desiredAccelerations;   // latest posted Drive() accelerations (stub, unread)
};

}  // namespace NKSwerve
```

### `SwerveDrive.cpp`

```cpp
#include "SwerveDrive.h"

#include <utility>

#include "wpi/util/array.hpp"

using namespace NKSwerve;

namespace {

std::array<SwerveModule, 4> BuildModules(SwerveDriveConfig& config) {
    return {
        SwerveModule(std::move(config.modules[0].driveMotor), std::move(config.modules[0].steerMotor)),
        SwerveModule(std::move(config.modules[1].driveMotor), std::move(config.modules[1].steerMotor)),
        SwerveModule(std::move(config.modules[2].driveMotor), std::move(config.modules[2].steerMotor)),
        SwerveModule(std::move(config.modules[3].driveMotor), std::move(config.modules[3].steerMotor)),
    };
}

wpi::math::SwerveDriveKinematics<4> BuildKinematics(const SwerveDriveConfig& config) {
    return wpi::math::SwerveDriveKinematics<4>{
        config.modules[0].position, config.modules[1].position,
        config.modules[2].position, config.modules[3].position};
}

wpi::util::array<wpi::math::SwerveModulePosition, 4> WpiModulePositionsOf(
    const std::array<SwerveModule, 4>& modules) {
    return wpi::util::array<wpi::math::SwerveModulePosition, 4>{
        modules[0].GetPosition(), modules[1].GetPosition(),
        modules[2].GetPosition(), modules[3].GetPosition()};
}

}  // namespace

SwerveDrive::SwerveDrive(SwerveDriveConfig config, std::shared_ptr<IGyroSource> gyro,
                         std::shared_ptr<IMutex> commandMutex)
    : m_config(std::move(config)),
      m_gyro(std::move(gyro)),
      m_modules(BuildModules(m_config)),
      m_kinematics(BuildKinematics(m_config)),
      m_odometry(m_kinematics, m_gyro->GetRotation2d(), WpiModulePositionsOf(m_modules)),
      m_commandMutex(std::move(commandMutex)),
      m_desiredVelocities(),
      m_desiredAccelerations() {}

void SwerveDrive::ThreadFunction() {
    // Deferred to a later pass: what this does with the posted
    // velocities/accelerations (m_desiredVelocities, m_desiredAccelerations)
    // -- kinematics -> per-module SetDesiredState, and the odometry update
    // -- isn't fixed yet. Intentionally a no-op for now.
}

void SwerveDrive::Drive(wpi::math::ChassisVelocities desiredVelocities,
                        wpi::math::ChassisAccelerations desiredAccelerations) {
    LockGuard lock(*m_commandMutex);
    m_desiredVelocities = desiredVelocities;
    m_desiredAccelerations = desiredAccelerations;
}

wpi::math::Rotation2d SwerveDrive::GetHeading() const {
    return m_gyro->GetRotation2d();
}

void SwerveDrive::ResetHeading() {
    m_gyro->Reset();
}

void SwerveDrive::ResetPose(wpi::math::Pose2d pose) {
    m_odometry.ResetPosition(m_gyro->GetRotation2d(), WpiModulePositionsOf(m_modules), pose);
}

wpi::math::Pose2d SwerveDrive::GetPose() const {
    return m_odometry.GetPose();
}

std::array<wpi::math::SwerveModulePosition, 4> SwerveDrive::GetModulePositions() const {
    return {m_modules[0].GetPosition(), m_modules[1].GetPosition(),
            m_modules[2].GetPosition(), m_modules[3].GetPosition()};
}

wpi::math::ChassisVelocities SwerveDrive::GetRobotRelativeVelocities() const {
    wpi::util::array<wpi::math::SwerveModuleVelocity, 4> velocities{
        m_modules[0].GetCurrentVelocity(), m_modules[1].GetCurrentVelocity(),
        m_modules[2].GetCurrentVelocity(), m_modules[3].GetCurrentVelocity()};
    return m_kinematics.ToChassisVelocities(velocities);
}

wpi::math::ChassisAccelerations SwerveDrive::GetRobotRelativeAccelerations() const {
    return {};
}
```

## Verification plan

1. From `NKSwerve/`, run `./gradlew NKSwerveWindowsx86-64DebugStaticLibrary`. Expected: succeeds, producing the `.lib` archive under `NKSwerve/build/`.
2. Run `./gradlew NKSwerveWindowsx86-64ReleaseStaticLibrary` for the release variant too.
3. Do **not** run `./gradlew build` or anything touching `NKSwerveTest` — that still fails on the unrelated `/std:c++23preview` toolchain issue, which is explicitly out of scope here.
4. If compilation fails anywhere, the failure must be a genuine mistake in this spec's rename (e.g., a missed header, wrong method name) — every type/method used above was confirmed against the real installed headers, so an error here means something in this document is wrong and needs fixing, not a pre-existing/expected condition to work around.

## Open items for future specs

- `SwerveDrive::ThreadFunction()`'s real behavior (still deferred, same as every prior spec).
- Real logic import from `archive/src/main/cpp/subsystems/SwerveDrive.cpp`/`SwerveModule.cpp` once this compiles, per the user's stated plan ("we will add tests later when we import the logic from the archive code").
- Whether `GetCurrentAcceleration()`/`GetRobotRelativeAccelerations()` ever get real implementations (e.g., derivative-based estimation) or stay permanent stubs.
- The `/std:c++23preview` `NKSwerveTest` toolchain issue, whenever test coverage is added back.
