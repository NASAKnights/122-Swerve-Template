// Copyright (c) FRC Team 122. All Rights Reserved.

#pragma once

#include <array>

#include <studica/AHRS.h>
#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/sim/Pigeon2SimState.hpp>
#include <ctre/phoenix6/StatusSignal.hpp>
#include <frc/RobotBase.h>
#include <frc/SPI.h>
#include <frc/controller/PIDController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Quaternion.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SubsystemBase.h>
#include <iostream>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/NetworkTableInstance.h>
#include <string>
#include <wpi/array.h>
#include <frc/Timer.h>
#include "subsystems/PoseEstimator.h"
#include "utils/POIGenerator.h"
#include "utils/PoseFilter.h"

#include <frc/DriverStation.h>
#include <frc/estimator/PoseEstimator.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/trajectory/constraint/SwerveDriveKinematicsConstraint.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
// #include <pathplanner/lib/util/PIDConstants.h>
// #include <pathplanner/lib/util/ReplanningConfig.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>

#include <units/angle.h>
#include <units/time.h>
#include <units/angular_velocity.h>
#include <units/math.h>

#include "Constants.hpp"
#include "SwerveModule.hpp"

class SwerveDrive : public frc2::SubsystemBase
{
public:
  SwerveDrive();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void SimulationPeriodic();

  void Drive(frc::ChassisSpeeds);

  void SetFast();
  void SetSlow();

  frc::Rotation2d GetHeading();
  void ResetHeading();
  void ResetDriveEncoders();
  void EnableDrive();
  void DisableDrive();
  // void InitPreferences();
  // void GetPrefernces();

  std::array<frc::SwerveModulePosition, 4> GetModulePositions();

  void ResetPose(frc::Pose2d position);

  frc::Pose2d GetPose();

  void UpdateOdometry();
  frc::ChassisSpeeds getRobotRelativeSpeeds();

  void InitializePID();
  void SetReference(frc::Pose2d);
  void Strafe(frc::ChassisSpeeds speeds, double angle);

  void UpdatePoseEstimate();
  void PublishOdometry(frc::Pose2d);
  void PrintNetworkTablseValues();
  void SetVision();
  bool atSetpoint();
  frc::Pose2d GetVision();
  void TurnVisionOn();
  void TurnVisionOff();
  void PeriodicShuffleboard();
  void ShuffleboardInit();
  void SetOffsets();
  void WeightedDriving(bool approach, double leftXAxis, double leftYAxis, double rightXAxis, std::string poiKey); // DEPRECATED

private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  // studica::AHRS m_gyro{frc::SPI::Port::kMXP};
  studica::AHRS navx{studica::AHRS::NavXComType::kMXP_SPI};

  // ctre::phoenix6::hardware::Pigeon2 m_pigeon{2};
  ctre::phoenix6::hardware::Pigeon2 m_pigeon{2, "NKCANivore"};
  // ctre::phoenix6::hardware::Pigeon2 m_pigeon{2};

  std::array<SwerveModule, 4> modules;
  frc::SwerveDriveKinematics<4U> kSwerveKinematics;

  frc::ChassisSpeeds speeds;
  frc::Field2d m_field;
  frc::PIDController pidX;
  frc::PIDController pidY;
  frc::PIDController pidRot;

  bool hasRun = false;
  bool enable = true;
  double pos_Error;

  double prevOError;

  bool useVision = true;

  POIGenerator poiGenerator;

  frc::ChassisSpeeds priorSpeeds = frc::ChassisSpeeds();

  nt::NetworkTableInstance networkTableInst;

  std::string_view baseLink1 = "base_link_1";
  std::string_view baseLink2 = "base_link_2";
  std::string_view baseLink = "base_link";
  std::string_view visionStdDev = "vision_stddev";
  std::string_view timeLinkName = "time";

  std::shared_ptr<nt::NetworkTable> poseTable;

  nt::DoubleArraySubscriber baseLink1Subscribe;
  nt::DoubleArraySubscriber baseLink2Subscribe;
  nt::DoubleArraySubscriber visionStdDevSub;
  PoseFilter poseFilter1 = PoseFilter(5, 0.2, 0.2);
  PoseFilter poseFilter2 = PoseFilter(5, 0.2, 0.2);
  frc::Quaternion rotation_q; // w, x, y, z
  frc::SwerveDrivePoseEstimator<4> m_poseEstimator;
  frc::Timer timer;
  PoseEstimator m_visionPoseEstimator;

  nt::DoubleArrayPublisher baseLinkPublisher;
  nt::DoubleArrayPublisher timePublisher;

  /* Simulation */
  frc::Timer m_simTimer;
  ctre::phoenix6::sim::Pigeon2SimState m_pigeonSim;
};
