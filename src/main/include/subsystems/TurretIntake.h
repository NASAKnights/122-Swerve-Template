// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>
#include <ctre/phoenix6/configs/Configs.hpp>

class TurretIntake : public frc2::SubsystemBase
{
public:
  TurretIntake();

  void Intake();
  void Outtake();
  void StopIntake();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

private:
  ctre::phoenix::motorcontrol::can::VictorSPX m_intakeMotor{8};

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
