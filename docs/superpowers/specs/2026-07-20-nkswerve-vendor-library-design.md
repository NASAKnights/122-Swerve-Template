# NKSwerve Vendor Library — Design Spec

## Context

This repository (`122-Swerve-Template`) currently mimics an independent FRC competition
robot project for team 122 (NASA Knights): a monolithic `src/` tree with a swerve
drivetrain plus several competition-specific subsystems (Climber, Elevator,
LEDController/LED_Groups, Turret, TurretIntake, Turret_Shooter, Wrist), vision-assisted
pose estimation, PathPlanner autonomous wiring, and a `PathCalibrator/` Python tool.

The goal is to turn this repo into a reusable **vendor library template** for FRC
robot codebases, structurally mirroring
[`wpilibsuite/vendor-template`](https://github.com/wpilibsuite/vendor-template)
(the `2027` branch), focused only on the C++ side (no Java/JNI).

This spec covers **phase 1 only: the importable library, `NKSwerve`.** A second,
separate spec will cover the consuming/testbed robot project (a full deployable
robot, controller-driven, movement-only, used to validate `NKSwerve` end-to-end via
a real vendordep install).

## Goals

- Produce `NKSwerve/`, a standalone C++ vendor library for swerve drivetrain control,
  structured like `wpilibsuite/vendor-template`'s `2027` branch (adapted to drop the
  Java/JNI driver split, since there's no Java consumer).
- Strip out everything that isn't core drivetrain logic: no other subsystems, no
  vision fusion, no PathPlanner wiring, no telemetry/Shuffleboard, no POI/pathfinding.
- Fully decouple the library from any specific hardware vendor. Motors, steer
  actuators, and the gyro are all injected via interfaces; `NKSwerve` itself depends
  on WPILib only (no CTRE Phoenix6, no REVLib).
- Fully decouple the library from any specific synchronization primitive too — the
  mutex guarding `SwerveDrive`'s command "notice board" is injected via an `IMutex`
  interface, not hardcoded to a WPILib (`wpi::mutex`/`wpi::priority_mutex`) type.
- Ship the library with unit test coverage (GoogleTest) for its pure logic.
- Leave the repo able to publish `NKSwerve` to `mavenLocal()` as a real vendordep,
  ready for a consuming robot project (built in the follow-up spec) to install.

## Non-goals (this spec)

- The consuming/testbed robot project (`testbed/`) — separate spec.
- Any concrete hardware implementation of the injected interfaces (e.g. a
  `Pigeon2Gyro`, `TalonFXDriveMotor`, `TalonFXSteerMotor`) — those belong to the
  testbed spec, since `NKSwerve` must not depend on CTRE/REV/any vendor library.
- CI (GitHub Actions) for building/publishing `NKSwerve` — can be a fast follow-up.
- Deleting the current repo-root monolithic robot project content — deferred to
  implementation of this spec's plan (not done during spec-writing), and is itself
  one of the plan's steps (see "Migration" below). It is **not** postponed to the
  second spec. `src/` has already been relocated to `archive/src/` (not deleted) so
  its logic isn't lost before extraction happens — see "Migration" below.

## Repo layout after this spec

```
122-Swerve-Template/
├── NKSwerve/
│   ├── build.gradle
│   ├── config.gradle
│   ├── publish.gradle
│   ├── settings.gradle
│   ├── gradlew / gradlew.bat / gradle/wrapper/...
│   ├── NKSwerve.json                  (vendordep JSON template)
│   ├── src/main/native/cpp/
│   │   ├── SwerveDrive.cpp
│   │   └── SwerveModule.cpp
│   ├── src/main/native/include/
│   │   ├── SwerveDrive.h
│   │   ├── SwerveModule.h
│   │   ├── SwerveDriveConfig.h
│   │   ├── IGyroSource.h
│   │   ├── IDriveMotor.h
│   │   ├── ISteerMotor.h
│   │   └── IMutex.h
│   └── src/test/native/cpp/
│       └── (GoogleTest unit tests)
├── README.md                          (rewritten for the vendor library; drops the
│                                        old competition changelog)
└── WPILib-License.md
```

Everything else present at the repo root today — `vendordeps/`, `build.gradle`,
`gradle.properties`, `settings.gradle`, `.wpilib/`, `.vscode/`, `.Glass/`,
`PathCalibrator/`, `networktables.json.bck` — is deleted as part of implementing this
spec's plan. `src/` isn't in that list: it's already been moved to `archive/src/`
ahead of the plan (preserving today's `SwerveDrive`/`SwerveModule` logic, and
everything else, for reference while `NKSwerve/` is built) rather than deleted
outright; `archive/` itself is scratch, not a permanent part of the repo, and can be
removed once `NKSwerve/` is verified against it — that's a separate decision, not
part of this spec. `testbed/` (and its own `build.gradle`/`vendordeps/`/etc.) is
added alongside `NKSwerve/` by the follow-up spec, not this one.

## Library design

### Namespace

All library types live under `NKSwerve::`.

### Hardware abstraction interfaces

`NKSwerve` depends on WPILib only. All actual hardware access is injected by the
consuming robot through three interfaces:

- **`IGyroSource`** — pure-virtual: `GetRotation2d() const`, `Reset()`.
- **`IDriveMotor`** — pure-virtual, operating in wheel-level units (the concrete
  implementation owns gear ratio / wheel circumference conversion internally):
  `SetVelocity(units::meters_per_second_t)`, `GetPosition() const` (wheel distance),
  `GetVelocity() const`, `ResetPosition(units::meter_t position = 0_m)`.
- **`ISteerMotor`** — pure-virtual: `SetAngle(frc::Rotation2d target)` (closed-loop
  position control), `GetAbsoluteRotation() const` (current absolute angle — the
  concrete implementation is responsible for however it derives an absolute reading,
  e.g. a fused/remote CANcoder configured on a TalonFX).

None of the three carry a `SimulationPeriodic` hook. Each interface exists only to
supply `SwerveModule`/`SwerveDrive` with what they need to command and read real
hardware; stepping a physics simulation is not something the module ever asks its
injected hardware to do, so it isn't part of this contract. A concrete
implementation is free to offer its own simulation-stepping method — it just isn't
called through `IGyroSource`/`IDriveMotor`/`ISteerMotor`, and `NKSwerve` neither
knows about it nor calls it (see the testbed spec for how the consuming robot drives
that).

There is no `ISteerEncoder` interface — absolute angle sensing is folded into
`ISteerMotor::GetAbsoluteRotation()`, since on modern FRC hardware (e.g. CTRE
Phoenix6 TalonFX with fused CANcoder feedback) the steer motor controller reports
absolute position directly.

No concrete implementations of any of these three interfaces ship in `NKSwerve`.
They are provided entirely by the consuming robot (the follow-up testbed spec).

### Synchronization abstraction

`SwerveDrive` holds a one-slot "notice board" internally — the most recently posted
`ChassisSpeeds` from `Drive()` (not a queue). This board is pure internal state, not
something a consuming robot needs to supply, so unlike the hardware sources it is
**not** exposed as an interface — `NKSwerve`'s interfaces exist only for objects a
consuming robot must actually provide (hardware access, or a platform-specific
primitive). The one piece of the board that *does* need to come from outside is the
lock guarding it, since the right implementation is platform-specific (a
priority-inheriting mutex matters on the RoboRIO's RT kernel; a unit test doesn't
need real locking at all). So `NKSwerve` defines a minimal, injectable locking
interface — deliberately not `wpi::mutex`/`wpi::priority_mutex` (WPILib) or a
hardcoded `std::mutex`. `SwerveDrive::ThreadFunction()` is what reads the board — the
one function meant to be called repeatedly by whatever the consuming application
uses to drive it (e.g. wired into its own `frc2::SubsystemBase::Periodic()` for a
command-based robot, or called some other way otherwise — `NKSwerve` has no opinion
on that). Deliberately not named `Periodic()`: it's a plain repeatedly-called
function, not a WPILib-framework override. What it actually does once it reads the
board — the functional behavior — is deferred to a second pass; this section only
fixes the structure.

```cpp
class IMutex {
public:
    virtual ~IMutex() = default;
    virtual void Lock() = 0;
    virtual void Unlock() = 0;
};

class LockGuard {
public:
    explicit LockGuard(IMutex& mutex) : m_mutex(mutex) { m_mutex.Lock(); }
    ~LockGuard() { m_mutex.Unlock(); }
    LockGuard(const LockGuard&) = delete;
    LockGuard& operator=(const LockGuard&) = delete;

private:
    IMutex& m_mutex;
};
```

`LockGuard` is `NKSwerve`'s own RAII helper (mirroring `std::lock_guard`) and depends
on nothing but `IMutex`. `SwerveDrive` takes an `std::shared_ptr<IMutex>` at
construction (same injection pattern as `IGyroSource`) and uses it to guard the
notice board. No concrete `IMutex` ships in `NKSwerve`; the consuming robot registers
one — e.g. a `wpi::priority_mutex`-backed implementation on real hardware, or a
trivial `std::mutex`-backed (or single-threaded no-op) implementation in unit tests
and host builds.

### `SwerveModule` (plain class, not a `frc2::SubsystemBase`)

Constructed from injected hardware:

```cpp
SwerveModule(std::unique_ptr<IDriveMotor> driveMotor,
             std::unique_ptr<ISteerMotor> steerMotor);
```

Responsibilities: `GetCurrentState() const`, `GetPosition() const`,
`SetDesiredState(frc::SwerveModuleState desiredState)` (includes state optimization
to avoid >90° steer rotation), `ResetDriveEncoder()`, `GetAbsoluteRotation() const`.
No thread function of its own — `SwerveModule` is driven synchronously, called
directly by `SwerveDrive::ThreadFunction()`.

Note: today's codebase has `SwerveModule` extend `frc2::SubsystemBase`, which
produces four independent command-based subsystems for what is really one
drivetrain's internals. In `NKSwerve`, neither `SwerveModule` nor `SwerveDrive` is a
`frc2::SubsystemBase` — `NKSwerve` has no WPILib command-based framework dependency
at all. Its only WPILib dependency is WPILib's math/geometry/units surface (`units::*`,
`frc::Pose2d`, `frc::Rotation2d`, `frc::Translation2d`, `frc::ChassisSpeeds`,
`frc::SwerveModuleState`/`SwerveModulePosition`, `frc::SwerveDriveKinematics`,
`frc::SwerveDriveOdometry`), never `frc2::` command-based types.

### `SwerveDriveConfig`

```cpp
struct SwerveModuleHardware {
    std::unique_ptr<IDriveMotor> driveMotor;
    std::unique_ptr<ISteerMotor> steerMotor;
    frc::Translation2d position;   // module location relative to robot center, for kinematics
};

struct SwerveDriveConfig {
    std::array<SwerveModuleHardware, 4> modules;   // FL, FR, BL, BR
    units::meters_per_second_t maxTranslationalVelocity;
    units::radians_per_second_t maxRotationalVelocity;
};
```

No CAN IDs, gear ratios, wheel diameters, current limits, or PID/feedforward gains
appear anywhere in `NKSwerve`. Those are all properties of a specific motor/gyro
implementation and live in the concrete classes the consuming robot supplies.

### `SwerveDrive` (plain class, not a `frc2::SubsystemBase`)

```cpp
class SwerveDrive {
public:
  SwerveDrive(SwerveDriveConfig config, std::shared_ptr<IGyroSource> gyro,
              std::shared_ptr<IMutex> commandMutex);

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
  frc::ChassisSpeeds m_desiredSpeeds;   // the notice board: latest posted Drive() value
};
```

`Drive(speeds)` takes `m_commandMutex`, overwrites `m_desiredSpeeds`, releases the
lock — that's the only behavior fixed for it here. `ThreadFunction()` is the only
function that reads `m_desiredSpeeds`; exactly what it does with `m_kinematics`,
`m_modules`, and `m_odometry` from there is functional behavior, left to a second
pass.

Uses plain `frc::SwerveDriveOdometry` — no vision pose fusion.

### Explicitly dropped from today's `SwerveDrive`/`SwerveModule`

Vision pose fusion (`PoseEstimator` subsystem, `PoseFilter`, NetworkTables
subscribers/publishers for vision), PathPlanner `AutoBuilder`/`RobotConfig`/
`PPHolonomicDriveController` wiring, `POIGenerator` and all POI/pathfinding code,
all Shuffleboard/SmartDashboard telemetry, `SetFast`/`SetSlow`, `EnableDrive`/
`DisableDrive`, `WeightedDriving`, offset-persistence-to-`frc::Preferences` logic,
the unused `studica::AHRS` navX field (only the Pigeon2 was actually driving heading
today — and now neither is referenced directly, since the gyro is behind
`IGyroSource`), and `SDSModuleType` (SDS module gear-ratio presets move to whichever
concrete `IDriveMotor`/`ISteerMotor` implementation wants them, in the testbed spec).
Also dropped entirely: Climber, Elevator, LEDController/LED_Groups, Turret,
TurretIntake, Turret_Shooter, Wrist, and the `AutoWheelOffsets` command — none of
these belong in a drivetrain-only vendor library.

## Build & publish

### `NKSwerve/build.gradle`

Adapted from `wpilibsuite/vendor-template`'s `2027` branch, trimmed to a single
library:

- Plugins: `cpp`, `google-test`, `org.wpilib.WPILibRepositoriesPlugin`,
  `org.wpilib.NativeUtils`, `org.wpilib.GradleVsCode`. No `java`, no
  `org.wpilib.GradleJni`.
- One `NKSwerve(NativeLibrarySpec)` component sourcing `src/main/native/cpp` /
  `src/main/native/include`, linked only against `wpilib_shared`. No driver
  component, no `privateExportsConfigs`.
- One `NKSwerveTest(GoogleTestTestSuiteSpec)` test suite sourcing
  `src/test/native/cpp`.
- `wpilibVersion = "2027.+"`, using `wpilibRepositories.use2027Repos()`.

### `NKSwerve/config.gradle`

Carried over close to verbatim from the upstream template — WPILib-generic
toolchain/cross-compile wiring (systemcore, linuxarm64, debug-binary suffixing),
nothing vendor-specific to change.

### `NKSwerve/publish.gradle`

Adapted to publish the single native-library artifact (headers + binaries) instead
of the upstream template's two (native + driver). GroupId `org.nasaknights`,
artifactId `NKSwerve-cpp`.

### `NKSwerve/NKSwerve.json` (vendordep template)

Single `cppDependencies` entry (`libName: "NKSwerve"`). No `javaDependencies`, no
`jniDependencies`.

### Publish loop

`./gradlew publishToMavenLocal` run inside `NKSwerve/` writes the published
artifacts into the local Maven cache, ready for a consuming robot project (the
follow-up testbed spec) to install via a normal `vendordeps/*.json` +
`mavenLocal()` repository entry — the same mechanism a real external team would use
to install any FRC vendor library, just pointed at the local cache instead of a
remote Maven host.

## Testing (this spec's scope)

`NKSwerveTest` (GoogleTest, `src/test/native/cpp`) covers only pure logic, using
simple in-test implementations of `IGyroSource`/`IDriveMotor`/`ISteerMotor`/`IMutex`
as test doubles — no hardware, no sim GUI, no deployable robot. The `IMutex` double is
a trivial `std::mutex`-backed (or single-threaded no-op) implementation; the tests run
on a host, not the RoboRIO's RT kernel, so there's no priority-inheritance behavior to
validate:

- `SwerveModule` state optimization (steer-target flips on >90° cases) and
  `SetDesiredState`/`GetCurrentState` wiring against fake motors.
- `SwerveDrive::Drive(ChassisSpeeds)` correctly posts to the notice board (readable
  back through whatever accessor the second pass adds) under `m_commandMutex`.
- Kinematics wiring and odometry update math (fake gyro, fake module hardware) once
  the second pass fixes what `ThreadFunction()` does when it consumes the board —
  deferred along with that pass.

Hardware-in-the-loop and sim-GUI validation (driving the swerve drive with a real
controller, in sim or on real hardware) is explicitly out of scope here — that is
the purpose of the follow-up testbed spec.

## Migration plan (executed during implementation of this spec, not during spec-writing)

0. Done ahead of the plan (not a spec-writing-time exception like the rest of this
   list — a real move already made in the repo): `src/` was relocated to
   `archive/src/` in its entirety, so today's `SwerveDrive`/`SwerveModule` logic (and
   everything else in `src/`) is preserved for reference and isn't lost when the
   repo root gets cleaned up in step 2.
1. Extract and rewrite the swerve-relevant logic from
   `archive/src/main/cpp/subsystems/SwerveDrive.cpp` and `SwerveModule.cpp` (and
   their headers) into `NKSwerve/`, per the class design above.
2. Delete everything else at the repo root: `vendordeps/`, `build.gradle`,
   `gradle.properties`, `settings.gradle`, `.wpilib/`, `.vscode/`, `.Glass/`,
   `PathCalibrator/`, `networktables.json.bck`, `gradlew`/`gradlew.bat`/`gradle/`
   (replaced by `NKSwerve/`'s own copies). `src/` is not in this list — it's already
   out of the way in `archive/src/`.
3. Rewrite `README.md` to describe `NKSwerve` as a vendor library (drop the old
   competition changelog).
4. Verify `NKSwerve/` builds standalone (`./gradlew build`), unit tests pass
   (`./gradlew test`), and it publishes cleanly (`./gradlew publishToMavenLocal`).

## Open items for the follow-up spec (consuming/testbed robot)

- Concrete `Pigeon2Gyro`, `TalonFXDriveMotor`, `TalonFXSteerMotor` implementations
  (owning CAN IDs, gear ratios, current limits, PID/feedforward gains, and CANcoder
  fused-feedback wiring).
- `testbed/` as a standard GradleRIO robot project (matching the shape of today's
  repo-root `build.gradle`, minus everything unrelated to swerve), installing both
  `NKSwerve.json` and CTRE's `Phoenix6.json` vendordeps.
- Controller-driven, movement-only teleop `Robot.cpp` (no autonomous, no other
  subsystems), targeting both desktop simulation and real hardware.
