# NKSwerve Testbed Robot — Design Spec

## Context

This is phase 2 of converting `122-Swerve-Template` from a monolithic competition
robot project into a `wpilibsuite/vendor-template`-style C++ vendor library. Phase 1
(see
[`2026-07-20-nkswerve-vendor-library-design.md`](2026-07-20-nkswerve-vendor-library-design.md))
produces `NKSwerve/`: a fully hardware-abstracted swerve drivetrain library
(`IGyroSource`/`IDriveMotor`/`ISteerMotor` interfaces injected into `SwerveModule`/
`SwerveDrive`), publishable to `mavenLocal()` as a real vendordep, with unit-test
coverage but no hardware-in-the-loop or sim validation.

This spec covers `testbed/`: a full, deployable FRC robot project that installs
`NKSwerve` the same way any external team would, provides concrete hardware
implementations of its interfaces, and drives via a controller for movement only.
Its purpose is to prove `NKSwerve` is actually usable end-to-end — both as a
distribution mechanism (real vendordep install) and as a piece of code (it actually
drives a robot, in sim and on real hardware).

## Goal

`testbed/` should look and feel exactly like an ordinary FRC team's robot project
that decided to adopt `NKSwerve` — not a special test harness. Standard GradleRIO
project layout, a real vendordep install (no Gradle multi-project shortcuts), a
`DriveSubsystem` wrapper a team would plausibly write themselves, and a
`Constants.h` a team would edit for their own robot.

## Non-goals

- No autonomous, no PathPlanner, no vision, no other subsystems — controller-driven
  movement only.
- No changes to `NKSwerve/` itself (phase 1 is already complete/frozen by this spec).
- No CI. No `RobotContainer` class — kept as simple as the scope allows, matching
  "just for movement."

## Repo layout

```
testbed/
├── build.gradle / settings.gradle          (standard GradleRIO robot project)
├── gradlew / gradlew.bat / gradle/wrapper/...
├── vendordeps/
│   ├── NKSwerve.json                        (installed from NKSwerve/'s publishToMavenLocal output)
│   ├── Phoenix6.json                        (CTRE TalonFX/CANcoder/Pigeon2)
│   └── WPILibNewCommands.json
├── src/main/include/
│   ├── Robot.h
│   ├── Constants.h
│   ├── subsystems/
│   │   └── DriveSubsystem.h
│   └── hardware/
│       ├── Pigeon2Gyro.h
│       ├── TalonFXDriveMotor.h
│       ├── TalonFXSteerMotor.h
│       └── WpiPriorityMutex.h
└── src/main/cpp/
    ├── Robot.cpp
    ├── subsystems/
    │   └── DriveSubsystem.cpp
    └── hardware/
        ├── Pigeon2Gyro.cpp
        ├── TalonFXDriveMotor.cpp
        ├── TalonFXSteerMotor.cpp
        └── WpiPriorityMutex.cpp
```

`testbed/` is added alongside `NKSwerve/` at the repo root (which by this point
contains only `NKSwerve/`, per phase 1's migration plan).

## Concrete hardware implementations (`src/main/{cpp,include}/hardware/`)

These are the only place in the repo that touches CTRE Phoenix6 — `NKSwerve/`
itself has zero vendor dependencies.

### `Pigeon2Gyro : NKSwerve::IGyroSource`

- Constructed with a CAN ID + CAN bus name (from `Constants.h`).
- `GetRotation2d()` wraps `Pigeon2::GetYaw()`.
- `Reset()` calls `Pigeon2::SetYaw(0)`.
- `SimulationPeriodic(rotationRate, dt)` advances `ctre::phoenix6::sim::Pigeon2SimState`
  by integrating the commanded rotational rate.

### `TalonFXDriveMotor : NKSwerve::IDriveMotor`

- Owns one `TalonFX` and its own config (CAN ID/bus, gear ratio, wheel diameter,
  inversion, neutral mode, current limits, drive PID/feedforward kP/kS/kV/kA — all
  sourced from `Constants.h`).
- `SetVelocity(units::meters_per_second_t)` converts wheel m/s to motor rotations/s
  via gear ratio + wheel circumference and issues a Phoenix6 `VelocityVoltage`
  control request (not `VelocityTorqueCurrentFOC`, to avoid requiring a Phoenix Pro
  license for the drive motors specifically).
- `GetPosition()`/`GetVelocity()` convert the TalonFX's reported rotor
  position/velocity back to wheel meters / meters-per-second.
- `SimulationPeriodic(velocity, dt)` drives a `frc::sim::DCMotorSim` (motor model,
  gear ratio, and moment of inertia from `Constants.h`) and feeds the result back
  into the TalonFX's `SimState`.

### `TalonFXSteerMotor : NKSwerve::ISteerMotor`

- Owns a `TalonFX` (steer) and a `CANcoder` (absolute reference); CAN IDs, encoder
  offset, and inversion from `Constants.h`.
- Team 122 holds Phoenix Pro licenses, so this configures the TalonFX's
  `FeedbackConfigs` with `FeedbackSensorSource::FusedCANcoder` pointing at the
  `CANcoder` — the TalonFX then reports absolute, continuous position directly with
  no manual sync step.
- `SetAngle(frc::Rotation2d target)` issues a Phoenix6 `PositionVoltage` request
  against the fused position.
- `GetAbsoluteRotation()` reads the TalonFX's fused position directly.
- `SimulationPeriodic(angle)` drives a `frc::sim::DCMotorSim` for the steer
  gearbox and feeds both the TalonFX's and the CANcoder's `SimState` so fused
  feedback stays consistent in simulation.

### `WpiPriorityMutex : NKSwerve::IMutex`

Not CTRE-related (unlike the three above) — it's the concrete lock `DriveSubsystem`
registers with `NKSwerve::SwerveDrive` for its internal command notice board (see the
vendor library spec's "Synchronization abstraction"). A thin wrapper around
`wpi::priority_mutex`, so the lock guarding `SwerveDrive`'s posted `ChassisSpeeds` gets
priority inheritance on the RoboRIO's RT kernel: `Lock()`/`Unlock()` forward directly
to the underlying `wpi::priority_mutex`'s `lock()`/`unlock()`.

## `Constants.h`

Generic placeholder values (explicitly **not** team 122's real hardware numbers —
just realistic-looking example values a team would replace with their own),
organized in namespaces mirroring today's `ElectricalConstants`/`DriveConstants`
pattern, trimmed to swerve-only:

```cpp
namespace ElectricalConstants {
    // Front-left, front-right, back-left, back-right: drive, steer, CANcoder
    constexpr int kFrontLeftDriveMotorId = 10, kFrontLeftSteerMotorId = 11, kFrontLeftCANcoderId = 12;
    constexpr int kFrontRightDriveMotorId = 20, kFrontRightSteerMotorId = 21, kFrontRightCANcoderId = 22;
    constexpr int kBackLeftDriveMotorId = 30, kBackLeftSteerMotorId = 31, kBackLeftCANcoderId = 32;
    constexpr int kBackRightDriveMotorId = 40, kBackRightSteerMotorId = 41, kBackRightCANcoderId = 42;
    constexpr int kPigeonId = 2;
    constexpr const char* kCanBus = "rio";
}

namespace DriveConstants {
    constexpr auto kTrackWidth = 0.6_m;
    constexpr auto kWheelBase = 0.6_m;
    constexpr auto kMaxTranslationalVelocity = 4.5_mps;
    constexpr auto kMaxRotationalVelocity = 4.0_rad_per_s;
    constexpr double kDriveGearRatio = 6.75;
    constexpr double kSteerGearRatio = 12.8;
    constexpr auto kWheelDiameter = 0.1016_m;
    constexpr double kDefaultAxisDeadband = 0.1;
    // per-module steer offsets, drive/steer PID+feedforward gains, and current
    // limits follow the same pattern — example values, not tuned/real ones.
}
```

## `DriveSubsystem` (`src/main/{cpp,include}/subsystems/`)

`DriveSubsystem : public frc2::SubsystemBase` — the one subsystem `Robot.cpp`
actually talks to. Its constructor builds four `TalonFXDriveMotor`/
`TalonFXSteerMotor` pairs, a `Pigeon2Gyro`, and a `WpiPriorityMutex` from
`Constants.h`, computes each module's `frc::Translation2d` from track width/wheelbase
(same ± pattern as today's `kFrontLeftPosition` etc.), assembles an
`NKSwerve::SwerveDriveConfig`, and constructs an internal `NKSwerve::SwerveDrive`
member (composition — the inner `SwerveDrive` is never itself registered with the
`CommandScheduler`) passing it the gyro and mutex.

Public surface: `Drive(frc::ChassisSpeeds)`, `GetPose()`, `ResetPose(frc::Pose2d)`,
`GetHeading()`, `ResetHeading()`. `DriveSubsystem::Periodic()` and
`SimulationPeriodic()` explicitly delegate into the inner `SwerveDrive`'s.

## Teleop control scheme

`Robot.cpp` — `frc::TimedRobot` + `frc2::CommandScheduler`, matching today's
pattern (idiomatic given `DriveSubsystem` is a `frc2::SubsystemBase`). No separate
`RobotContainer` class — binding stays inline in `Robot.cpp`, matching the
"movement-only" scope.

- One `frc::XboxController m_driverController` at port 0.
- `DriveSubsystem`'s default command:
  - Left stick Y/X → translational X/Y (field-relative by default), right stick X
    → rotation. All three axes go through an axis-deadband helper
    (`MathUtilNK::calculateAxis`, moved here from the old repo root — it's generic
    input processing, not drivetrain-library logic, so it belongs in `testbed`,
    not `NKSwerve`).
  - Holding the left bumper switches to robot-relative drive (mirrors today's
    button-5 behavior).
  - "A" button → `m_driveSubsystem.ResetHeading()`.
- `AutonomousInit`/`AutonomousPeriodic` are empty overrides — no autonomous
  command is ever scheduled.

## Build setup

`testbed/build.gradle` — a standard GradleRIO project (`cpp`,
`google-test-test-suite`, `edu.wpi.first.GradleRIO`, RoboRIO deploy target, desktop
sim enabled via `wpi.sim.addGui()`/`wpi.sim.addDriverstation()`), matching the
shape of today's repo-root `build.gradle` minus everything unrelated to swerve.
Pinned to a GradleRIO/WPILib version compatible with `NKSwerve`'s `2027.+` line.

`testbed/vendordeps/`:
- **`NKSwerve.json`** — installed the way any external team would: run
  `./gradlew publishToMavenLocal` inside `NKSwerve/`, add `mavenLocal()` to
  `testbed/build.gradle`'s repositories, drop the vendordep JSON in. No Gradle
  multi-project `include` shortcuts — this is the real distribution path.
- **`Phoenix6.json`** — needed here (not in `NKSwerve/`), since the concrete
  hardware classes live in `testbed`.
- **`WPILibNewCommands.json`** — carried over from today's setup.

## Simulation & telemetry

- Desktop sim works via GradleRIO's standard sim support, matching today's
  `build.gradle` sim config.
- `DriveSubsystem::SimulationPeriodic()` drives the physics-based `DCMotorSim`
  wiring described above, so controller input produces realistic simulated
  wheel/steer motion — not just a simple integrator.
- A small `frc::Field2d`, published to `SmartDashboard`/Glass and updated from
  `DriveSubsystem::GetPose()` each `RobotPeriodic()`, so movement can be visually
  confirmed in sim (translation direction, field-relative vs. robot-relative,
  rotation). This telemetry lives entirely in `testbed` — nothing is added back to
  `NKSwerve`, consistent with phase 1's "no telemetry in the library" decision.

## Validation plan

1. `./gradlew build` succeeds for both `NKSwerve/` (already done in phase 1) and
   `testbed/`, with `testbed` resolving `NKSwerve` purely through the
   `mavenLocal()` vendordep path.
2. Desktop sim: drive with an Xbox controller (or keyboard-mapped joystick in the
   sim GUI), confirm field-relative and robot-relative modes both behave
   correctly, confirm heading reset works, watch the `Field2d` to confirm pose
   tracking looks correct for known stick inputs.
3. Real hardware: deploy to an actual RoboRIO + the concrete TalonFX/CANcoder/
   Pigeon2 hardware described in `Constants.h` (values substituted for the real
   chassis being tested on), confirm the same controller-driven behavior on a real
   swerve chassis.
