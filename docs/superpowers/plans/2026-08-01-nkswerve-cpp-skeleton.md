# NKSwerve C++ Skeleton Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Create the structural C++ skeleton for `NKSwerve` (the hardware/synchronization interfaces, `SwerveModule`, and `SwerveDrive`) exactly as specified in [`2026-07-20-nkswerve-vendor-library-design.md`](../specs/2026-07-20-nkswerve-vendor-library-design.md), extracting real logic from the archived `archive/src/main/{cpp,include}/subsystems/SwerveDrive.*`/`SwerveModule.*` where the spec has fixed the behavior, and leaving `SwerveDrive::ThreadFunction()` as an explicit stub where the spec has deferred it.

**Architecture:** Nine files under `NKSwerve/src/main/native/{include,cpp}/`: four leaf interfaces (`IGyroSource`, `IDriveMotor`, `ISteerMotor`, `IMutex`) with no dependencies on each other, a `SwerveDriveConfig` struct, a plain `SwerveModule` class driven synchronously by its owner, and a plain `SwerveDrive` class that owns four `SwerveModule`s plus kinematics/odometry and exposes the mutex-guarded "notice board" described in the spec. No `frc2::SubsystemBase`, no build/Gradle scaffolding, no tests wired up — this pass is structure only, per explicit direction to not worry about working code yet.

**Tech Stack:** C++, WPILib (`frc::`/`units::` geometry, kinematics, odometry, units types only — no CTRE/REV, no command-based framework).

## Global Constraints

- No WPILib classes other than its math/geometry/units surface (`frc::Pose2d`, `frc::Rotation2d`, `frc::Translation2d`, `frc::ChassisSpeeds`, `frc::SwerveModuleState`/`SwerveModulePosition`, `frc::SwerveDriveKinematics`, `frc::SwerveDriveOdometry`, `units::*`) — never `frc2::` command-based types, never `wpi::mutex`/`wpi::priority_mutex`.
- All library types live under the `NKSwerve::` namespace.
- No `SimulationPeriodic` anywhere in these files — simulation stepping is not part of `NKSwerve`'s contract (see spec's "Hardware abstraction interfaces" section).
- `SwerveModule` and `SwerveDrive` are plain classes, not `frc2::SubsystemBase`.
- `SwerveDrive::ThreadFunction()` is a stub (empty body with an explanatory comment) — its behavior is explicitly deferred to a later pass. Every other method is real, working logic.
- **Known open risk, not resolved by this plan:** the only locally-installed WPILib matching this spec's declared `wpilibVersion = "2027.+"` (`2027_alpha5`, confirmed via its `wpimath-cpp` headers) uses a renamed C++ API — `wpi::math::` instead of `frc::`, `wpi::units::` instead of `units::`, `SwerveModuleVelocity`/`SwerveModuleAcceleration` instead of `SwerveModuleState`, `ChassisVelocities`/`ChassisAccelerations` instead of `ChassisSpeeds`, `wpi::util::array` instead of `wpi::array`/`std::array` — with no `frc::` compatibility alias found. This plan uses the spec's existing `frc::`/`units::` naming (matching the archived source and the spec as written) because this pass is structure-only and no build is attempted. Reconciling which namespace to actually target is required before any build-verification pass.
- No `.gitignore`/build.gradle/test-runner changes in this plan — `NKSwerve/` is new, untracked source only.

---

## Task 1: Hardware and synchronization interfaces

**Files:**
- Create: `NKSwerve/src/main/native/include/IGyroSource.h`
- Create: `NKSwerve/src/main/native/include/IDriveMotor.h`
- Create: `NKSwerve/src/main/native/include/ISteerMotor.h`
- Create: `NKSwerve/src/main/native/include/IMutex.h`

**Interfaces:**
- Consumes: nothing (leaf files, only WPILib/units headers).
- Produces (for Task 2 and Task 3):
  - `NKSwerve::IGyroSource` — `frc::Rotation2d GetRotation2d() const`, `void Reset()`.
  - `NKSwerve::IDriveMotor` — `void SetVelocity(units::meters_per_second_t)`, `units::meter_t GetPosition() const`, `units::meters_per_second_t GetVelocity() const`, `void ResetPosition(units::meter_t position = 0_m)`.
  - `NKSwerve::ISteerMotor` — `void SetAngle(frc::Rotation2d target)`, `frc::Rotation2d GetAbsoluteRotation() const`.
  - `NKSwerve::IMutex` — `void Lock()`, `void Unlock()`.
  - `NKSwerve::LockGuard` — RAII helper, `explicit LockGuard(IMutex& mutex)`.

- [ ] **Step 1: Create `IGyroSource.h`**

```cpp
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
```

- [ ] **Step 2: Create `IDriveMotor.h`**

```cpp
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
```

- [ ] **Step 3: Create `ISteerMotor.h`**

```cpp
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
```

- [ ] **Step 4: Create `IMutex.h`**

```cpp
#pragma once

namespace NKSwerve {

class IMutex {
public:
    virtual ~IMutex() = default;

    virtual void Lock() = 0;
    virtual void Unlock() = 0;
};

// NKSwerve's own RAII helper, mirroring std::lock_guard, depending on nothing
// but IMutex.
class LockGuard {
public:
    explicit LockGuard(IMutex& mutex) : m_mutex(mutex) { m_mutex.Lock(); }
    ~LockGuard() { m_mutex.Unlock(); }

    LockGuard(const LockGuard&) = delete;
    LockGuard& operator=(const LockGuard&) = delete;

private:
    IMutex& m_mutex;
};

}  // namespace NKSwerve
```

- [ ] **Step 5: Verify against the spec**

Read all four files back and confirm each method signature matches the "Hardware abstraction interfaces" and "Synchronization abstraction" sections of `docs/superpowers/specs/2026-07-20-nkswerve-vendor-library-design.md` exactly (method names, parameter types, default arguments, `const`-ness). There is no build system wired up yet in this pass, so this is a manual read-back, not a compiler check.

- [ ] **Step 6: Commit**

```bash
git add NKSwerve/src/main/native/include/IGyroSource.h NKSwerve/src/main/native/include/IDriveMotor.h NKSwerve/src/main/native/include/ISteerMotor.h NKSwerve/src/main/native/include/IMutex.h
git commit -m "Add NKSwerve hardware and synchronization interfaces"
```

---

## Task 2: SwerveModule

**Files:**
- Create: `NKSwerve/src/main/native/include/SwerveModule.h`
- Create: `NKSwerve/src/main/native/cpp/SwerveModule.cpp`

**Interfaces:**
- Consumes: `NKSwerve::IDriveMotor`, `NKSwerve::ISteerMotor` (Task 1, `IDriveMotor.h`/`ISteerMotor.h`).
- Produces (for Task 3):
  - `NKSwerve::SwerveModule` constructor: `SwerveModule(std::unique_ptr<IDriveMotor> driveMotor, std::unique_ptr<ISteerMotor> steerMotor)`.
  - `frc::SwerveModuleState GetCurrentState() const`
  - `frc::SwerveModulePosition GetPosition() const`
  - `void SetDesiredState(frc::SwerveModuleState desiredState)`
  - `void ResetDriveEncoder()`
  - `frc::Rotation2d GetAbsoluteRotation() const`

- [ ] **Step 1: Create `SwerveModule.h`**

```cpp
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
```

- [ ] **Step 2: Create `SwerveModule.cpp`**

This is the one file in this plan carrying real, non-stub domain logic (state optimization), ported from `archive/src/main/cpp/subsystems/SwerveModule.cpp:155-174`'s `SetDesiredState`, adapted to call through the injected interfaces instead of TalonFX/CANcoder directly, and to use `GetAbsoluteRotation()` (the new design's single source of truth for current angle) instead of the old code's separate relative-position `GetRotation()`.

```cpp
#include "SwerveModule.h"

#include <utility>

using namespace NKSwerve;

SwerveModule::SwerveModule(std::unique_ptr<IDriveMotor> driveMotor,
                           std::unique_ptr<ISteerMotor> steerMotor)
    : m_driveMotor(std::move(driveMotor)), m_steerMotor(std::move(steerMotor)) {}

frc::SwerveModuleState SwerveModule::GetCurrentState() const {
    return {m_driveMotor->GetVelocity(), m_steerMotor->GetAbsoluteRotation()};
}

frc::SwerveModulePosition SwerveModule::GetPosition() const {
    return {m_driveMotor->GetPosition(), m_steerMotor->GetAbsoluteRotation()};
}

void SwerveModule::SetDesiredState(frc::SwerveModuleState desiredState) {
    frc::Rotation2d currentAngle = m_steerMotor->GetAbsoluteRotation();
    desiredState = frc::SwerveModuleState::Optimize(desiredState, currentAngle);

    m_steerMotor->SetAngle(desiredState.angle);
    m_driveMotor->SetVelocity(desiredState.speed);
}

void SwerveModule::ResetDriveEncoder() {
    m_driveMotor->ResetPosition();
}

frc::Rotation2d SwerveModule::GetAbsoluteRotation() const {
    return m_steerMotor->GetAbsoluteRotation();
}
```

- [ ] **Step 3: Verify against the spec and the archived reference**

Read both files back. Confirm the "Responsibilities" list in the spec's `SwerveModule` section is fully covered, confirm `SetDesiredState`'s optimize-then-command sequence matches the intent of `archive/src/main/cpp/subsystems/SwerveModule.cpp:155-166` (optimize against current angle, then command steer angle and drive velocity), and confirm there is no `Periodic()`/`SimulationPeriodic()` anywhere in either file. Manual read-back only — no build system wired up yet.

- [ ] **Step 4: Commit**

```bash
git add NKSwerve/src/main/native/include/SwerveModule.h NKSwerve/src/main/native/cpp/SwerveModule.cpp
git commit -m "Add NKSwerve SwerveModule"
```

---

## Task 3: SwerveDriveConfig and SwerveDrive

**Files:**
- Create: `NKSwerve/src/main/native/include/SwerveDriveConfig.h`
- Create: `NKSwerve/src/main/native/include/SwerveDrive.h`
- Create: `NKSwerve/src/main/native/cpp/SwerveDrive.cpp`

**Interfaces:**
- Consumes: `NKSwerve::IGyroSource`, `NKSwerve::IDriveMotor`, `NKSwerve::ISteerMotor` (Task 1), `NKSwerve::IMutex`/`NKSwerve::LockGuard` (Task 1), `NKSwerve::SwerveModule` (Task 2, constructor + `GetCurrentState()`/`GetPosition()`).
- Produces: `NKSwerve::SwerveDriveConfig` (used by the future testbed spec's `DriveSubsystem`), `NKSwerve::SwerveDrive` — the library's public entry point:
  - `SwerveDrive(SwerveDriveConfig config, std::shared_ptr<IGyroSource> gyro, std::shared_ptr<IMutex> commandMutex)`
  - `void ThreadFunction()` — **stub, no-op** (functional behavior deferred)
  - `void Drive(frc::ChassisSpeeds speeds)`
  - `frc::Rotation2d GetHeading() const`
  - `void ResetHeading()`
  - `void ResetPose(frc::Pose2d pose)`
  - `frc::Pose2d GetPose() const`
  - `std::array<frc::SwerveModulePosition, 4> GetModulePositions() const`
  - `frc::ChassisSpeeds GetRobotRelativeSpeeds() const`

- [ ] **Step 1: Create `SwerveDriveConfig.h`**

```cpp
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
```

- [ ] **Step 2: Create `SwerveDrive.h`**

```cpp
#pragma once

#include <array>
#include <memory>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModulePosition.h>

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
    // Deliberately not named Periodic() — a plain repeatedly-called function,
    // not a WPILib-framework override. What it does with the posted
    // ChassisSpeeds once read is deferred to a later pass; currently a no-op.
    void ThreadFunction();

    void Drive(frc::ChassisSpeeds speeds);

    frc::Rotation2d GetHeading() const;
    void ResetHeading();

    void ResetPose(frc::Pose2d pose);
    frc::Pose2d GetPose() const;

    std::array<frc::SwerveModulePosition, 4> GetModulePositions() const;
    frc::ChassisSpeeds GetRobotRelativeSpeeds() const;

private:
    SwerveDriveConfig m_config;
    std::shared_ptr<IGyroSource> m_gyro;
    std::array<SwerveModule, 4> m_modules;
    frc::SwerveDriveKinematics<4> m_kinematics;
    frc::SwerveDriveOdometry<4> m_odometry;

    std::shared_ptr<IMutex> m_commandMutex;
    frc::ChassisSpeeds m_desiredSpeeds;  // the notice board: latest posted Drive() value
};

}  // namespace NKSwerve
```

- [ ] **Step 3: Create `SwerveDrive.cpp`**

Construction order matters: class members initialize in declaration order (`m_config`, `m_gyro`, `m_modules`, `m_kinematics`, `m_odometry`, `m_commandMutex`, `m_desiredSpeeds`), so `m_config` must be initialized first (moved from the constructor parameter) before `m_modules` can move the injected motors out of it, and `m_modules`/`m_kinematics` must exist before `m_odometry` reads initial module positions and heading. Three small anonymous-namespace helpers keep the initializer list readable. Real logic ported from `archive/src/main/cpp/subsystems/SwerveDrive.cpp`: `GetHeading`/`ResetHeading` from lines 247-260, `ResetPose`/`GetPose` from lines 277-286 (using `frc::SwerveDriveOdometry` directly instead of the old `SwerveDrivePoseEstimator`, since this design has no vision fusion), `GetModulePositions` from lines 270-275, and `GetRobotRelativeSpeeds` from lines 292-299 (`getRobotRelativeSpeeds` there).

```cpp
#include "SwerveDrive.h"

#include <utility>

#include <wpi/array.h>

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

frc::SwerveDriveKinematics<4> BuildKinematics(const SwerveDriveConfig& config) {
    return frc::SwerveDriveKinematics<4>{
        config.modules[0].position, config.modules[1].position,
        config.modules[2].position, config.modules[3].position};
}

wpi::array<frc::SwerveModulePosition, 4> WpiModulePositionsOf(
    const std::array<SwerveModule, 4>& modules) {
    return wpi::array<frc::SwerveModulePosition, 4>{
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
      m_desiredSpeeds() {}

void SwerveDrive::ThreadFunction() {
    // Deferred to a later pass: what this does with the posted ChassisSpeeds
    // (m_desiredSpeeds) — kinematics -> per-module SetDesiredState, and the
    // odometry update — isn't fixed yet. Intentionally a no-op for now.
}

void SwerveDrive::Drive(frc::ChassisSpeeds speeds) {
    LockGuard lock(*m_commandMutex);
    m_desiredSpeeds = speeds;
}

frc::Rotation2d SwerveDrive::GetHeading() const {
    return m_gyro->GetRotation2d();
}

void SwerveDrive::ResetHeading() {
    m_gyro->Reset();
}

void SwerveDrive::ResetPose(frc::Pose2d pose) {
    m_odometry.ResetPosition(m_gyro->GetRotation2d(), WpiModulePositionsOf(m_modules), pose);
}

frc::Pose2d SwerveDrive::GetPose() const {
    return m_odometry.GetPose();
}

std::array<frc::SwerveModulePosition, 4> SwerveDrive::GetModulePositions() const {
    return {m_modules[0].GetPosition(), m_modules[1].GetPosition(),
            m_modules[2].GetPosition(), m_modules[3].GetPosition()};
}

frc::ChassisSpeeds SwerveDrive::GetRobotRelativeSpeeds() const {
    wpi::array<frc::SwerveModuleState, 4> states{
        m_modules[0].GetCurrentState(), m_modules[1].GetCurrentState(),
        m_modules[2].GetCurrentState(), m_modules[3].GetCurrentState()};
    return m_kinematics.ToChassisSpeeds(states);
}
```

- [ ] **Step 4: Verify against the spec**

Read all three files back. Confirm the `SwerveDrive` public method list matches the spec's class code block exactly (including that `ThreadFunction()` has no parameters and no return value), confirm the private member declaration order matches the spec (`m_config`, `m_gyro`, `m_modules`, `m_kinematics`, `m_odometry`, `m_commandMutex`, `m_desiredSpeeds`) since that order is what makes the constructor initializer list valid C++, and confirm `Drive()` is the only method that writes `m_desiredSpeeds` and does so under `LockGuard`. Manual read-back only — no build system wired up yet.

- [ ] **Step 5: Commit**

```bash
git add NKSwerve/src/main/native/include/SwerveDriveConfig.h NKSwerve/src/main/native/include/SwerveDrive.h NKSwerve/src/main/native/cpp/SwerveDrive.cpp
git commit -m "Add NKSwerve SwerveDriveConfig and SwerveDrive"
```

---

## Self-Review

**Spec coverage:**
- Hardware abstraction interfaces (`IGyroSource`, `IDriveMotor`, `ISteerMotor`) — Task 1. ✓
- Synchronization abstraction (`IMutex`, `LockGuard`) — Task 1. ✓
- `SwerveModule` (plain class, state optimization, no thread function) — Task 2. ✓
- `SwerveDriveConfig` — Task 3, Step 1. ✓
- `SwerveDrive` (plain class, notice board, `ThreadFunction()`, plain accessors) — Task 3, Steps 2-3. ✓
- No `SimulationPeriodic` anywhere — enforced in Global Constraints and verification steps of every task. ✓
- Gradle/build scaffolding, root-directory cleanup, README rewrite, build verification (migration plan steps 2-4) — explicitly out of scope for this plan; not covered here.

**Placeholder scan:** No "TBD"/"TODO"/"implement later" in any step. `ThreadFunction()`'s empty body is not a placeholder — it's the spec's own explicit, named deferral, documented as such in both the header comment and Global Constraints, not a gap in this plan's coverage.

**Type consistency:** `SwerveModule`'s constructor signature (Task 2, Step 1) matches what Task 3's `BuildModules` helper calls. `IDriveMotor`/`ISteerMotor`/`IGyroSource`/`IMutex` method names used in Task 2 and Task 3's `.cpp` files match Task 1's declarations exactly (`GetVelocity`, `GetPosition`, `SetVelocity`, `ResetPosition`, `GetAbsoluteRotation`, `SetAngle`, `GetRotation2d`, `Reset`, `Lock`, `Unlock`). `SwerveDrive`'s private member list in Task 3 Step 2 matches the order Step 3's constructor initializer list uses.

## Execution Handoff

Plan complete and saved to `docs/superpowers/plans/2026-08-01-nkswerve-cpp-skeleton.md`. Two execution options:

1. **Subagent-Driven (recommended)** - I dispatch a fresh subagent per task, review between tasks, fast iteration
2. **Inline Execution** - Execute tasks in this session using executing-plans, batch execution with checkpoints

Which approach?
