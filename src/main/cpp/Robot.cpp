// Copyright (c) FRC Team 122. All Rights Reserved.

#include "Robot.hpp"

Robot::Robot() : networkTableInst(nt::NetworkTableInstance::GetDefault())
{
    this->CreateRobot();
}

// This function is called during startup
void Robot::RobotInit()
{
    frc::DataLogManager::Start();
    wpi::log::DataLog &log = frc::DataLogManager::GetLog();
    m_VoltageLog = wpi::log::DoubleLogEntry(log, "/PDP/Voltage");
    m_CurrentLog = wpi::log::DoubleLogEntry(log, "/PDP/Current");
    m_PowerLog = wpi::log::DoubleLogEntry(log, "/PDP/Power");
    m_EnergyLog = wpi::log::DoubleLogEntry(log, "/PDP/Energy");
    m_TemperatureLog = wpi::log::DoubleLogEntry(log, "/PDP/Temperature");
    m_BatteryLog = wpi::log::DoubleLogEntry(log, "Robot/Battery");

    frc::SmartDashboard::PutString("POIName", "");
    frc::SmartDashboard::PutData("AddPOI", addPOICommand.get());
    frc::SmartDashboard::PutData("RemovePOI", removePOICommand.get());
    frc::SmartDashboard::PutData("Set", autoWheelOffsetsCommand.get());

    autoChooser = pathplanner::AutoBuilder::buildAutoChooser();

    frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);

    auto sdTable = networkTableInst.GetTable("SmartDashboard");
    modelPosePublisher = sdTable->GetStructArrayTopic<frc::Pose3d>("ModelPoses").Publish();
};

// This function is called every 20 ms
void Robot::RobotPeriodic()
{
    frc2::CommandScheduler::GetInstance().Run();
    this->UpdateDashboard();
    m_VoltageLog.Append(m_pdh.GetVoltage());
    m_CurrentLog.Append(m_pdh.GetTotalCurrent());
    m_PowerLog.Append(m_pdh.GetTotalPower());
    m_EnergyLog.Append(m_pdh.GetTotalEnergy());
    m_TemperatureLog.Append(m_pdh.GetTemperature());
    m_BatteryLog.Append(batteryShunt.GetVoltage());

    frc::Pose2d pose = frc::Pose2d(units::length::meter_t{0.0}, units::length::meter_t{0.0}, frc::Rotation2d{});

    // frc::Pose3d stageOne3dPOS = frc::Pose3d(pose.X(), pose.Y(), units::length::meter_t(m_elevator.GetHeight()) / 2, frc::Rotation3d(pose.Rotation()));
    // frc::Pose3d carage3dPOS = frc::Pose3d(pose.X(), pose.Y(), units::length::meter_t(m_elevator.GetHeight()), frc::Rotation3d(pose.Rotation()));
    // frc::Pose3d wrist3dPOS = frc::Pose3d(0.28_m, 0_m, units::length::meter_t(m_elevator.GetHeight() + 0.595), frc::Rotation3d(units::angle::radian_t{0.0}, units::angle::radian_t{-m_wrist.GetMeasurement()}, units::angle::radian_t{0.0}));
    // frc::Pose3d climb3dPOS = frc::Pose3d(0_m, 0_m, 0_m, frc::Rotation3d(0.0_rad, 0.0_rad, 0.0_rad));
    // std::vector<frc::Pose3d> modelPoses = {
    //     stageOne3dPOS,
    //     carage3dPOS,
    //     wrist3dPOS,
    //     climb3dPOS,
    // };
    // modelPosePublisher.Set(modelPoses, 0);
}

// This function is called once each time the robot enters Disabled mode.
void Robot::DisabledInit()
{
    // m_LED_Controller.DefaultAnimation();
}

void Robot::SetAutonomousCommand(std::string a)
{
}

void Robot::AutonomousInit()
{
    // m_autonomousCommand = this->GetAutonomousCommand();
    // m_elevator.HoldPosition();
    // m_swerveDrive.TurnVisionOff(); // don't use vision during Auto
    auto m_autonomousCommand = autoChooser.GetSelected();
    m_swerveDrive.ResetPose(autoStartPose);

    if (m_autonomousCommand)
    {
        m_autonomousCommand->Schedule();
    }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit()
{
    // m_elevator.Disable();
}

void Robot::TeleopInit()
{
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    // m_wrist.HoldPosition();
    // m_elevator.HoldPosition();
    /*
    if (m_wrist.GetState() != WristConstants::WristState::ZEROING)
    {
        m_wrist.SetAngle(m_wrist.GetMeasurement().value());
    }
    */

    if (m_autonomousCommand)
    {
        m_autonomousCommand->Cancel();
    }
    m_swerveDrive.TurnVisionOn(); // Turn Vision back on for Teleop
    // m_LED_Controller.TeleopLED();
}

void Robot::TeleopPeriodic()
{
    // if (m_elevator.GetHeight() >= 0.35 && !m_pathfind.IsScheduled())
    // {
    //     frc::SmartDashboard::PutNumber("drive/accelLim", 0.5);
    // }
    // else
    // {
    //     frc::SmartDashboard::PutNumber("drive/accelLim", 4.0);
    // }
}

void Robot::TeleopExit()
{
    // m_elevator.Disable();
}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

/**
 * This function is called once when the robot is first started up.
 */
void Robot::SimulationInit() {}

/**
 * This function is called periodically whilst in simulation.
 */
void Robot::SimulationPeriodic() {}

/**
 * Initializes the robot subsystems and binds commands
 */
void Robot::CreateRobot()
{
    // NOTE: THIS WAS FOR REEFSCAPE PSEDUO-AUTO ALIGNMENT WITH THE REEF,
    //  WE SHOULD LATER ATTEMPT TO SEPARATE THIS FROM THE ROBOT.CPP AND MAKE IT MORE FLEXABLE FOR MORE GENERAL ALIGNMENT TO POI's
    //  scoreClosest = frc2::CommandPtr(
    //      frc2::cmd::RunOnce(
    //          [&]()
    //          {
    //              using namespace pathplanner;
    //              using namespace frc;
    //              Pose2d currentPose = this->m_swerveDrive.GetPose();
    //              // Select Left or Right Branch
    //              frc::Transform2d offset = m_driverController.GetRawButton(7) ?
    //                  frc::Transform2d(0.0_m, 0.35_m, frc::Rotation2d()) :
    //                  frc::Transform2d(0.0_m, 0.0_m, frc::Rotation2d());

    //             // The rotation component in these poses represents the direction of travel
    //             Pose2d startPos = Pose2d(currentPose.Translation(), Rotation2d());
    //             Pose2d endPos = m_poiGenerator.GetClosestPOI().TransformBy(offset);

    //             auto transformedEndPos = endPos.TransformBy(Transform2d(0.25_m, 0_m, 0_rad));
    //             std::vector<Waypoint> waypoints = PathPlannerPath::waypointsFromPoses({startPos, endPos, transformedEndPos});
    //             // Paths must be used as shared pointers
    //             auto path = std::make_shared<PathPlannerPath>(
    //                 waypoints,
    //                 std::vector<RotationTarget>({RotationTarget(0.25, endPos.Rotation())}),
    //                 std::vector<PointTowardsZone>(),
    //                 std::vector<ConstraintsZone>(),
    //                 std::vector<EventMarker>(),
    //                 PathConstraints(1_mps, 1.5_mps_sq, 360_deg_per_s, 940_deg_per_s_sq),
    //                 // PathConstraints(1_mps, 2.0_mps_sq, 360_deg_per_s, 940_deg_per_s_sq),
    //                 std::nullopt, // Ideal starting state can be nullopt for on-the-fly paths
    //                 GoalEndState(0_mps, endPos.Rotation()),
    //                 false
    //             );

    //             // Prevent this path from being flipped on the red alliance, since the given positions are already correct
    //             path->preventFlipping = true;

    //             m_pathfind = frc2::CommandPtr(AutoBuilder::followPath(path).Unwrap());
    //             m_pathfind.Schedule(); })
    //         .Unwrap());

    m_swerveDrive.SetDefaultCommand(frc2::RunCommand(
        [this]
        {
            auto controllerIn = m_driverController.GetRawButton(5);
            // bool approach = 0;

            auto leftXAxis = MathUtilNK::calculateAxis(m_driverController.GetRawAxis(1),
                                                       DriveConstants::kDefaultAxisDeadband);
            auto leftYAxis = MathUtilNK::calculateAxis(m_driverController.GetRawAxis(0),
                                                       DriveConstants::kDefaultAxisDeadband);
            auto rightXAxis = MathUtilNK::calculateAxis(m_driverController.GetRawAxis(2),
                                                        DriveConstants::kDefaultAxisDeadband);

            // m_swerveDrive.WeightedDriving(approach, leftXAxis, leftYAxis, rightXAxis, targetKey);

            if (controllerIn)
                // Robot-Oriented Drive
                m_swerveDrive.Drive(frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                    -leftXAxis * DriveConstants::kMaxTranslationalVelocity,
                    -leftYAxis * DriveConstants::kMaxTranslationalVelocity,
                    -rightXAxis * DriveConstants::kMaxRotationalVelocity, frc::Rotation2d()));
            else
            {
                m_swerveDrive.Drive(frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                    -leftXAxis * DriveConstants::kMaxTranslationalVelocity,
                    -leftYAxis * DriveConstants::kMaxTranslationalVelocity,
                    -rightXAxis * DriveConstants::kMaxRotationalVelocity, m_swerveDrive.GetHeading()));
            }
        },
        {&m_swerveDrive}));

    // AddPeriodic([this]
    //             { m_elevator.Periodic(); },
    //             5_ms, 1_ms);
    // AddPeriodic([this]
    //             { m_wrist.Periodic(); },
    //             10_ms, 2_ms);

    // Configure the button bindings
    BindCommands();
    m_swerveDrive.ResetHeading();
    // m_LED_Controller.DefaultAnimation();
}

/**
 * Binds commands to Joystick buttons
 */
void Robot::BindCommands()
{

    // --------------DRIVER BUTTONS----------------------------------
    frc2::JoystickButton(&m_driverController, 1)
        .OnTrue(frc2::CommandPtr(
            frc2::InstantCommand([this]
                                 { return m_swerveDrive.ResetHeading(); })));

    frc2::JoystickButton(&m_driverController, 3)
        .OnTrue(scoreClosest.get())
        .OnFalse(frc2::CommandPtr(
            frc2::InstantCommand([this]
                                 { return m_pathfind.Cancel(); })));

    // --------------OPERATOR BUTTONS--------------------------------

    // frc2::POVButton(&m_operatorController, 0) // Zero wrist
    //     .OnTrue(frc2::CommandPtr(frc2::InstantCommand(
    //         [this]
    //         {
    //             m_elevator.Zero();
    //             return;
    //         })));
}

void Robot::DisabledPeriodic()
{
    std::string poiName = std::string("POI/") + frc::SmartDashboard::GetString("POIName", "");
    frc::SmartDashboard::PutBoolean("IsPersist", frc::SmartDashboard::IsPersistent(poiName));
}

void Robot::UpdateDashboard()
{
    frc::SmartDashboard::PutNumber("Robot/Battery Amps", batteryShunt.GetVoltage());
    frc::SmartDashboard::PutNumber("Robot/PDH Total Current", m_pdh.GetTotalCurrent());
}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
