// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Turret_Shooter.h"

Turret_Shooter::Turret_Shooter()
{

    mainShooterMotorConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);
    followerShooterMotorConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);
    backShooterMotorConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);

    mainShooterMotorConfig.SmartCurrentLimit(30);
    followerShooterMotorConfig.SmartCurrentLimit(30);
    backShooterMotorConfig.SmartCurrentLimit(30);

    mainShooterMotorConfig.Inverted(true);

    // mainShooterMotorConfig.closedLoop
    //     .P(kP)
    //     .I(kI)
    //     .D(kD)
    //     .OutputRange(kMinOutput, kMaxOutput);
    // followerShooterMotorConfig.closedLoop
    //     .P(kP)
    //     .I(kI)
    //     .D(kD)
    //     .OutputRange(kMinOutput, kMaxOutput);
    // backShooterMotorConfig.closedLoop
    //     .P(kP * 10)
    //     .I(kI)
    //     .D(kD)
    //     .OutputRange(kMinOutput, kMaxOutput);

    m_followerShooterMotor.Configure(followerShooterMotorConfig, rev::spark::SparkBase::ResetMode::kNoResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);
    m_mainShooterMotor.Configure(mainShooterMotorConfig, rev::spark::SparkBase::ResetMode::kNoResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);
    m_backMotor.Configure(backShooterMotorConfig, rev::spark::SparkBase::ResetMode::kNoResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);

    frc::SmartDashboard::PutNumber("Shooter_Speed", shooterSpeed);
}

void Turret_Shooter::SetSpeed()
{

    shooterSpeed = frc::SmartDashboard::GetNumber("Shooter_Speed", 0.0);

    if (min_speed <= shooterSpeed <= max_speed)
    {
        mainMotorController.SetReference(shooterSpeed, rev::spark::SparkLowLevel::ControlType::kVelocity);
        backMotorController.SetReference(shooterSpeed, rev::spark::SparkLowLevel::ControlType::kVelocity);
    }
    else if (min_speed > shooterSpeed)
    {
        mainMotorController.SetReference(min_speed, rev::spark::SparkLowLevel::ControlType::kVelocity);
        backMotorController.SetReference(min_speed, rev::spark::SparkLowLevel::ControlType::kVelocity);
    }
    else if (max_speed < shooterSpeed)
    {
        mainMotorController.SetReference(max_speed, rev::spark::SparkLowLevel::ControlType::kVelocity);
        backMotorController.SetReference(max_speed, rev::spark::SparkLowLevel::ControlType::kVelocity);
    }

    // mainMotorController.SetReference(shooterSpeed, rev::spark::SparkLowLevel::ControlType::kVelocity);
}

void Turret_Shooter::StopMotors()
{
    shooterSpeed = 0.0;
    mainMotorController.SetReference(0, rev::spark::SparkLowLevel::ControlType::kVelocity);
    backMotorController.SetReference(0, rev::spark::SparkLowLevel::ControlType::kVelocity);
    m_mainShooterMotor.Set(0.);
    m_backMotor.Set(0.);
}

void Turret_Shooter::NewSetSpeed()
{
    newShooterSpeed = frc::SmartDashboard::GetNumber("Shooter_Speed", 0.0);

    if (m_mainShooterMotor.GetEncoder().GetVelocity() >= newShooterSpeed)
    {
        // StopMotors();
        m_mainShooterMotor.Set(0.);
    }
    else if (m_mainShooterMotor.GetEncoder().GetVelocity() < newShooterSpeed)
    {
        m_mainShooterMotor.Set(0.9);
    }

    if (m_backMotor.GetEncoder().GetVelocity() >= newShooterSpeed)
    {
        // StopMotors();
        m_backMotor.Set(0.475);
    }
    else if (m_backMotor.GetEncoder().GetVelocity() < newShooterSpeed)
    {
        m_backMotor.Set(0.9);
        // Controls a motor with the output of the BangBang controller and a feedforward
        // Shrinks the feedforward slightly to avoid overspeeding the shooter
        // m_backMotor.SetVoltage(0.9 * m_feedforward.Calculate(newShooterSpeed));
    }
}
// This method will be called once per scheduler run
void Turret_Shooter::Periodic()
{

    frc::SmartDashboard::PutNumber("Shooter/actual speed", m_mainShooterMotor.GetEncoder().GetVelocity());
}
