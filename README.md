# 122 Reefscape 2025 Code Base

This is the code base for FRC 2025 Reefscape Game for Team 122 NASA Knights

# General Robot Specs

TBD: ???


# Changelog

### Updated 2/22/25

- Intake for both Coral and Algae have been merged into main
- LED Controller has been merged into main
- Current version of Visual Servoing has been merged into main
- Climber Subsystem has been merged into main

### Updated 2/1/25

- Elevator code has been merged into main
- Elevator code works with running the elevator in the correct direction but with no load at the moment
- PID values for swerve steering have been tuned
- Some libraries have been updated to new version

### Updated 1/28/25

- Persistent Constants only work with the 2024 version of Shuffleboard, network tables are still intact but interaction with values differ between 2025 Shuffleboard and 2024 Shuffleboard
- Skeleton code of various commands generated and added. Will need to modify in the future as robot develops
- High level commands are written as sequential commands groups, state machines may still need to be implemented
- Initial swerve drive offsets have been generated, will need more calibration for autos. PID values need to be tuned.

### Updated 1/21/25

- Merged Persistent Constants branch into main
- Robot is now able to change wheels to different offsets and stay persistant between restarts/reboots
- Updated libraries

### Updated 1/11/25

- Updated to the new WPILib version 2025.2.1
- Removed the Util folder due to NKTrajectory and NKTrajectoryManager not working with the gradle build
- **Note:** New WPILib with cpp needs 17.9+ compiler, please update accordingly
- Updated Vendor libraries
- **Note:** Navx vendor library has changed to Studica
- Calibrate Function has been removed due to library changes
- How the Navx is instantiated is changed as well
#### CTRE Changes
- Pigeon has changed to GetYaw(), read specs as rotation direction has changed
- CurrentSupplyConfig values have changed definition, updated accordingly
- SupplyCurrentLowerLimit seems to be the new ContinuousCurrentLimit **TODO: CHECK**
- SupplyCurrentLimit seems to be the new PeakLimit **TODO: CHECK BEFORE RUNNING**
- SupplyCurrentLowerTime seems to be the new SupplyTimeThreshold **TODO: CHECK BEFORE RUNNING**
#### SwerveDriveModule
- Optimize function has changed
#### Pathplanner
- Changes to how the SwerveAutoBuilder is instantiated
- File paths for include have been updated and removed as necessary
