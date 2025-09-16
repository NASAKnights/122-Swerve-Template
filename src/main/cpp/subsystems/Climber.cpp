// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Climber.h"

Climber::Climber()
{

    climbMainConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
    climbMainConfig.SmartCurrentLimit(30);
    climbMainConfig.VoltageCompensation(12.0);

    climbFollowConfig.Follow(climbMain, true);
    climbFollowConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
    climbFollowConfig.SmartCurrentLimit(30);
    climbFollowConfig.VoltageCompensation(12.0);
    climbFollowConfig.absoluteEncoder.PositionConversionFactor(2 * std::numbers::pi);

    climbMain.Configure(climbMainConfig, rev::spark::SparkBase::ResetMode::kNoResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);
    climbFollower.Configure(climbFollowConfig, rev::spark::SparkBase::ResetMode::kNoResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);

    climbWristController.Reset();

    wpi::log::DataLog &log = frc::DataLogManager::GetLog();
    m_AbsolutePosition = wpi::log::DoubleLogEntry(log, "/Climber/AbsolutePos");
};

// This method will be called once per scheduler run
void Climber::Periodic()
{
    m_AbsolutePosition.Append(climberWristEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Climb Rotation", climberWristEncoder.GetPosition());
}

void Climber::Unspool()
{
    climbMain.Set(-0.4);
}

void Climber::Deploy()
{
    // Deploys the climber out
    // climbMain.Set(climbWristController.Calculate(climberWristEncoder.GetPosition(), kClimbDeploySetPoint));
    climbMain.Set(0.8);
}

void Climber::Climb()
{
    // Pulls the winch back in
    // climbMain.Set(climbWristController.Calculate(climberWristEncoder.GetPosition(), kClimbClimbSetPoint));
    climbMain.Set(0.8);
}

void Climber::Stop()
{
    climbMain.Set(0.0);
}

bool Climber::atClimbAngle()
{
    return climberWristEncoder.GetPosition() < Climber::kClimbClimbSetPoint;
}

bool Climber::atDeployAngle()
{
    return climberWristEncoder.GetPosition() > Climber::kClimbDeploySetPoint;
}
