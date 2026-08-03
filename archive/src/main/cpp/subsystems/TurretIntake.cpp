// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/TurretIntake.h"

TurretIntake::TurretIntake()
{
}

// This method will be called once per scheduler run
void TurretIntake::Periodic() {}

void TurretIntake::Intake()
{
    m_intakeMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.85);
}

void TurretIntake::Outtake()
{
    m_intakeMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.85);
}

void TurretIntake::StopIntake()
{
    m_intakeMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
}