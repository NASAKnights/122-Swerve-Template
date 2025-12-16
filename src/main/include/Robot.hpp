// Copyright (c) FRC Team 122. All Rights Reserved.

#pragma once

#include <optional>

#include "frc/DataLogManager.h"
#include "wpi/DataLog.h"
#include <frc/Joystick.h>
#include <frc/PowerDistribution.h>
#include <frc/TimedRobot.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/AnalogInput.h>
#include "utils/POIGenerator.h"

#include <frc2/command/CommandPtr.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/POVButton.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/AutoBuilder.h>

#include "subsystems/SwerveDrive.hpp"
#include "subsystems/Elevator.h"
#include "subsystems/Wrist.h"

#include <units/angular_velocity.h>
#include <units/velocity.h>

#include "subsystems/Elevator.h"
#include "subsystems/Wrist.h"

#include "subsystems/LEDController.h"
#include "subsystems/Climber.h"

#include "commands/AutoWheelOffsets.h"

#include <cmath>

class Robot : public frc::TimedRobot
{
public:
    Robot();

    //
    // Robot Schedule methods
    //
    void RobotInit() override;
    void RobotPeriodic() override;
    void DisabledInit() override;
    void DisabledPeriodic() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void AutonomousExit() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void TeleopExit() override;
    void TestPeriodic() override;
    void SimulationInit() override;
    void SimulationPeriodic() override;

private:
    // Have it empty by default so that if testing teleop it
    // doesn't have undefined behavior and potentially crash.
    std::optional<frc2::CommandPtr> m_autonomousCommand;

    std::map<int, std::pair<pathplanner::PathPlannerAuto, frc::Pose2d>> autoMap;

    // LEDController m_LED_Controller;

    // Subsystems

    frc::SendableChooser<std::string> m_chooser;
    frc::AnalogInput batteryShunt{0};

    SwerveDrive m_swerveDrive;
    // Wrist m_wrist;
    // Elevator m_elevator;
    // Climber m_climber;

    std::string_view baseLink = "base_link";
    nt::StructPublisher<frc::Pose3d> stageOne3dPOS;
    nt::StructPublisher<frc::Pose3d> carage3dPOS;
    nt::StructPublisher<frc::Pose3d> wrist3dPOS;
    nt::StructPublisher<frc::Pose3d> wristPOS;
    nt::StructPublisher<frc::Pose3d> climb3dPOS;
    nt::StructArrayPublisher<frc::Pose3d> modelPosePublisher;
    nt::NetworkTableInstance networkTableInst;

    std::string targetKey = "POI/Calibration POIs";
    std::string prevAuto = "";

    frc::PowerDistribution m_pdh =
        frc::PowerDistribution{1, frc::PowerDistribution::ModuleType::kRev};

    // PS4 controllers
    frc::Joystick m_driverController{DriveConstants::kDriverPort};
    frc::Joystick m_operatorController{DriveConstants::kOperatorPort};
    frc::Joystick m_DebugController{DriveConstants::kDebugControllerPort};

    // Power Distribution
    wpi::log::DoubleLogEntry m_VoltageLog;
    wpi::log::DoubleLogEntry m_CurrentLog;
    wpi::log::DoubleLogEntry m_PowerLog;
    wpi::log::DoubleLogEntry m_EnergyLog;
    wpi::log::DoubleLogEntry m_TemperatureLog;
    wpi::log::DoubleLogEntry m_BatteryLog;
    frc::SendableChooser<frc2::Command *> autoChooser;
    POIGenerator m_poiGenerator;
    frc2::CommandPtr m_pathfind = frc2::InstantCommand().ToPtr();
    frc2::CommandPtr scoreClosest = frc2::InstantCommand().ToPtr();

    frc2::CommandPtr addPOICommand = frc2::CommandPtr(frc2::InstantCommand([this]
                                                                           { return m_poiGenerator.MakePOI(); }))
                                         .IgnoringDisable(true);

    frc2::CommandPtr removePOICommand = frc2::CommandPtr(frc2::InstantCommand([this]
                                                                              { return m_poiGenerator.RemovePOI(); }))
                                            .IgnoringDisable(true);

    frc2::CommandPtr autoWheelOffsetsCommand = AutoWheelOffsets(&m_swerveDrive).ToPtr().IgnoringDisable(true);

    // Robot Container methods
    void CreateRobot();
    void BindCommands();

    frc::Pose2d autoStartPose;

    frc2::CommandPtr GetAutonomousCommand();
    void SetAutonomousCommand(std::string a);
    // std::function<void(std::string)> SetAutonomousCommand(std::string a);
    // void SetTAutonomousCommand(std::string a);
    void UpdateDashboard();
};
