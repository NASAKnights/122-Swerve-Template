// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/controller/PIDController.h>
#include "frc/DataLogManager.h"
#include "wpi/DataLog.h"
#include <rev/SparkFlex.h>
#include <rev/AbsoluteEncoder.h>
#include <rev/SparkAbsoluteEncoder.h>
#include <rev/SparkMax.h>
#include <frc/smartdashboard/SmartDashboard.h>

class Climber : public frc2::SubsystemBase
{
public:
  Climber();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void Deploy();
  void Climb();
  void Unspool();
  bool atDeployAngle();
  bool atClimbAngle();
  void Stop();

private:
  double kClimbP = 0.1;
  double kClimbI = 0.0;
  double kClimbD = 0.0;

  double kClimbDeploySetPoint = 0.79 * (2 * std::numbers::pi);
  double kClimbClimbSetPoint = 0.6 * (2 * std::numbers::pi); // radians

  rev::spark::SparkFlex climbMain{7, rev::spark::SparkLowLevel::MotorType::kBrushless};
  rev::spark::SparkFlex climbFollower{8, rev::spark::SparkLowLevel::MotorType::kBrushless};

  rev::spark::SparkClosedLoopController climbPID = climbMain.GetClosedLoopController();
  rev::spark::SparkBaseConfig climbMainConfig;
  rev::spark::SparkBaseConfig climbFollowConfig;

  // rev::spark::SparkMax absoluteEncoderSub{9, rev::spark::SparkLowLevel::MotorType::kBrushed};

  rev::spark::SparkAbsoluteEncoder climberWristEncoder = climbFollower.GetAbsoluteEncoder();
  // rev::spark::SparkFlexExternalEncoder climberWristEncoder = climbMain.GetExternalEncoder();
  // rev::spark::SparkRelativeEncoder climberWristEncoder = climbMain.GetEncoder();

  frc::PIDController climbWristController{kClimbP, kClimbI, kClimbD};

  // Maybe for the spark flex?
  // rev::spark::SparkFlexExternalEncoder climberWristFlexEncoder = climbMain.GetExternalEncoder();

  // Logging

  wpi::log::DoubleLogEntry m_AbsolutePosition;
};
