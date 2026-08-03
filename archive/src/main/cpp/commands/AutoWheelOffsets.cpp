// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoWheelOffsets.h"
#include <frc/smartdashboard/SmartDashboard.h>

AutoWheelOffsets::AutoWheelOffsets(SwerveDrive *swerve) : m_swerve{swerve}
{
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void AutoWheelOffsets::Initialize()
{
  // Module 1
  frc::SmartDashboard::PutNumber("FrontLeftDegree", 0);

  // Module 2
  frc::SmartDashboard::PutNumber("FrontRightDegree", 0);

  // Module 3
  frc::SmartDashboard::PutNumber("BackLeftDegree", 0);

  // Module 4
  frc::SmartDashboard::PutNumber("BackRightDegree", 0);

  m_swerve->SetOffsets();
  // double WheelOffsets = frc::SmartDashboard::GetNumber("Module " + std::to_string(1) + "/" + " Reported Angle", 0);
  // double WheelOffsets = frc::SmartDashboard::GetNumber("Module " + std::to_string(1) + "/ Reported Angle", 0);

  // frc::SmartDashboard::SetDefaultNumber("FrontLeftDegree 2", 0.1);

  // frc::SmartDashboard::PutNumber("FrontLeftDegree", WheelOffsets);
  // frc::SmartDashboard::PutNumber ("FrontLeftDegree", "Module " + std::to_string(m_id) + "/" + " Reported Angle");
  // frc::SmartDashboard::SetPersistent("FrontLeftDegree");

  // frc::Rotation2d kFrontLeftOffset(-units::degree_t{WheelOffsets});
}

// Called repeatedly when this Command is scheduled to run
void AutoWheelOffsets::Execute()
{
  // front Left (Module 1)
  double WheelOffsets = frc::SmartDashboard::GetNumber("Module " + std::to_string(1) + "/ CANCoder Angle", 0);
  frc::SmartDashboard::PutNumber("FrontLeftDegree", WheelOffsets);
  frc::Rotation2d kFrontLeftOffset(-units::degree_t{WheelOffsets});

  // front Right (Module 2)
  WheelOffsets = frc::SmartDashboard::GetNumber("Module " + std::to_string(2) + "/ CANCoder Angle", 0);
  frc::SmartDashboard::PutNumber("FrontRightDegree", WheelOffsets);
  frc::Rotation2d kFrontRightOffset(-units::degree_t{WheelOffsets});

  // back Left (Module 3)
  WheelOffsets = frc::SmartDashboard::GetNumber("Module " + std::to_string(3) + "/ CANCoder Angle", 0);
  frc::SmartDashboard::PutNumber("BackLeftDegree", WheelOffsets);
  frc::Rotation2d kBackLeftOffset(-units::degree_t{WheelOffsets});

  // back Right (Module 4)
  WheelOffsets = frc::SmartDashboard::GetNumber("Module " + std::to_string(4) + "/ CANCoder Angle", 0);
  frc::SmartDashboard::PutNumber("BackRightDegree", WheelOffsets);
  frc::Rotation2d kBackRightOffset(-units::degree_t{WheelOffsets});
  m_swerve->SetOffsets();
}

// Called once the command ends or is interrupted.
void AutoWheelOffsets::End(bool interrupted) {}

// Returns true when the command should end.
bool AutoWheelOffsets::IsFinished()
{
  return true;
}