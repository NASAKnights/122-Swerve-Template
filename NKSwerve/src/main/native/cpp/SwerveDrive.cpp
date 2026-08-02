#include "SwerveDrive.h"

#include <utility>

#include <wpi/array.h>

using namespace NKSwerve;

namespace {

std::array<SwerveModule, 4> BuildModules(SwerveDriveConfig& config) {
    return {
        SwerveModule(std::move(config.modules[0].driveMotor), std::move(config.modules[0].steerMotor)),
        SwerveModule(std::move(config.modules[1].driveMotor), std::move(config.modules[1].steerMotor)),
        SwerveModule(std::move(config.modules[2].driveMotor), std::move(config.modules[2].steerMotor)),
        SwerveModule(std::move(config.modules[3].driveMotor), std::move(config.modules[3].steerMotor)),
    };
}

frc::SwerveDriveKinematics<4> BuildKinematics(const SwerveDriveConfig& config) {
    return frc::SwerveDriveKinematics<4>{
        config.modules[0].position, config.modules[1].position,
        config.modules[2].position, config.modules[3].position};
}

wpi::array<frc::SwerveModulePosition, 4> WpiModulePositionsOf(
    const std::array<SwerveModule, 4>& modules) {
    return wpi::array<frc::SwerveModulePosition, 4>{
        modules[0].GetPosition(), modules[1].GetPosition(),
        modules[2].GetPosition(), modules[3].GetPosition()};
}

}  // namespace

SwerveDrive::SwerveDrive(SwerveDriveConfig config, std::shared_ptr<IGyroSource> gyro,
                         std::shared_ptr<IMutex> commandMutex)
    : m_config(std::move(config)),
      m_gyro(std::move(gyro)),
      m_modules(BuildModules(m_config)),
      m_kinematics(BuildKinematics(m_config)),
      m_odometry(m_kinematics, m_gyro->GetRotation2d(), WpiModulePositionsOf(m_modules)),
      m_commandMutex(std::move(commandMutex)),
      m_desiredSpeeds() {}

void SwerveDrive::ThreadFunction() {
    // Deferred to a later pass: what this does with the posted ChassisSpeeds
    // (m_desiredSpeeds) — kinematics -> per-module SetDesiredState, and the
    // odometry update — isn't fixed yet. Intentionally a no-op for now.
}

void SwerveDrive::Drive(frc::ChassisSpeeds speeds) {
    LockGuard lock(*m_commandMutex);
    m_desiredSpeeds = speeds;
}

frc::Rotation2d SwerveDrive::GetHeading() const {
    return m_gyro->GetRotation2d();
}

void SwerveDrive::ResetHeading() {
    m_gyro->Reset();
}

void SwerveDrive::ResetPose(frc::Pose2d pose) {
    m_odometry.ResetPosition(m_gyro->GetRotation2d(), WpiModulePositionsOf(m_modules), pose);
}

frc::Pose2d SwerveDrive::GetPose() const {
    return m_odometry.GetPose();
}

std::array<frc::SwerveModulePosition, 4> SwerveDrive::GetModulePositions() const {
    return {m_modules[0].GetPosition(), m_modules[1].GetPosition(),
            m_modules[2].GetPosition(), m_modules[3].GetPosition()};
}

frc::ChassisSpeeds SwerveDrive::GetRobotRelativeSpeeds() const {
    wpi::array<frc::SwerveModuleState, 4> states{
        m_modules[0].GetCurrentState(), m_modules[1].GetCurrentState(),
        m_modules[2].GetCurrentState(), m_modules[3].GetCurrentState()};
    return m_kinematics.ToChassisSpeeds(states);
}
