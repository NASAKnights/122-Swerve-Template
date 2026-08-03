# NKSwerve

A C++ vendor library for FRC swerve drivetrain control, structured like
[`wpilibsuite/vendor-template`](https://github.com/wpilibsuite/vendor-template)'s
`2027` branch. Motors, steer actuators, and the gyro are all injected via
interfaces (`IDriveMotor`, `ISteerMotor`, `IGyroSource`) — `NKSwerve` itself
depends on WPILib only, never a specific hardware vendor.

See [`docs/superpowers/specs/2026-07-20-nkswerve-vendor-library-design.md`](docs/superpowers/specs/2026-07-20-nkswerve-vendor-library-design.md)
for the full library design, and
[`docs/superpowers/specs/2026-08-03-nkswerve-build-scaffolding-migration-design.md`](docs/superpowers/specs/2026-08-03-nkswerve-build-scaffolding-migration-design.md)
for how the project is structured and built.

## Building

```bash
cd NKSwerve
./gradlew build
```

## Installing in a robot project

```bash
cd NKSwerve
./gradlew publishToMavenLocal
```

Then, in a consuming robot project's `build.gradle`, add `mavenLocal()` to
`repositories`, and drop `NKSwerve/build/repos/NKSwerve.json` into that
project's `vendordeps/` directory.

## License

See [`WPILib-License.md`](WPILib-License.md).
