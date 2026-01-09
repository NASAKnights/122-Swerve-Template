// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/SparkFlex.h>
#include <rev/SparkMax.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/voltage.h>
#include <units/moment_of_inertia.h>

#include <rev/SparkBase.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/PIDSubsystem.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/SimpleMotorFeedforward.h>

class Turret_Shooter : public frc2::SubsystemBase
{
public:
  Turret_Shooter();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void StopMotors();
  void SetSpeed();
  void NewSetSpeed();
  double max_speed = 5000; // need to change to actual value we want
  double min_speed = -5000;

private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  rev::spark::SparkFlex m_mainShooterMotor{13, rev::spark::SparkLowLevel::MotorType::kBrushless};
  rev::spark::SparkFlex m_followerShooterMotor{14, rev::spark::SparkLowLevel::MotorType::kBrushless};
  rev::spark::SparkMax m_backMotor{15, rev::spark::SparkLowLevel::MotorType::kBrushless};

  rev::spark::SparkBaseConfig followerShooterMotorConfig;
  rev::spark::SparkBaseConfig mainShooterMotorConfig;
  rev::spark::SparkBaseConfig backShooterMotorConfig;

  rev::spark::SparkClosedLoopController mainMotorController = m_mainShooterMotor.GetClosedLoopController();
  rev::spark::SparkClosedLoopController backMotorController = m_backMotor.GetClosedLoopController();

  // static constexpr auto kFFks = 0.05_V;                                                // Volts static (motor)
  // static constexpr auto kFFkV = 0.25_V / 1.0_rpm;                                      // volts*s/meters //1.01 // 2.23
  // static constexpr auto kFFkA = 0.38_V / units::revolutions_per_minute_squared_t{1.0}; // volts*s^2/meters //0.1
  // frc::SimpleMotorFeedforward<units::turn_t> m_feedforward;

  double shooterSpeed = 0.0;
  double newShooterSpeed = 0.0;

  double kP = 0.005;
  double kI = 0.0;
  double kD = 0.0;
  double kMinOutput = -1.0;
  double kMaxOutput = 1.0;
};
