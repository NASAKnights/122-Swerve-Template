#include "SwerveModule.h"

#include <utility>

using namespace NKSwerve;

SwerveModule::SwerveModule(std::unique_ptr<IDriveMotor> driveMotor,
                           std::unique_ptr<ISteerMotor> steerMotor)
    : m_driveMotor(std::move(driveMotor)), m_steerMotor(std::move(steerMotor)) {}

frc::SwerveModuleState SwerveModule::GetCurrentState() const {
    return {m_driveMotor->GetVelocity(), m_steerMotor->GetAbsoluteRotation()};
}

frc::SwerveModulePosition SwerveModule::GetPosition() const {
    return {m_driveMotor->GetPosition(), m_steerMotor->GetAbsoluteRotation()};
}

void SwerveModule::SetDesiredState(frc::SwerveModuleState desiredState) {
    frc::Rotation2d currentAngle = m_steerMotor->GetAbsoluteRotation();
    desiredState = frc::SwerveModuleState::Optimize(desiredState, currentAngle);

    m_steerMotor->SetAngle(desiredState.angle);
    m_driveMotor->SetVelocity(desiredState.speed);
}

void SwerveModule::ResetDriveEncoder() {
    m_driveMotor->ResetPosition();
}

frc::Rotation2d SwerveModule::GetAbsoluteRotation() const {
    return m_steerMotor->GetAbsoluteRotation();
}
