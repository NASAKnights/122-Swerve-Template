#pragma once

#include <array>
#include <memory>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModulePosition.h>

#include "IGyroSource.h"
#include "IMutex.h"
#include "SwerveDriveConfig.h"
#include "SwerveModule.h"

namespace NKSwerve {

// Plain class, not a frc2::SubsystemBase.
class SwerveDrive {
public:
    SwerveDrive(SwerveDriveConfig config, std::shared_ptr<IGyroSource> gyro,
                std::shared_ptr<IMutex> commandMutex);

    // The one function meant to be called repeatedly by the consuming
    // application (e.g. wired into its own frc2::SubsystemBase::Periodic()).
    // Deliberately not named Periodic() — a plain repeatedly-called function,
    // not a WPILib-framework override. What it does with the posted
    // ChassisSpeeds once read is deferred to a later pass; currently a no-op.
    void ThreadFunction();

    void Drive(frc::ChassisSpeeds speeds);

    frc::Rotation2d GetHeading() const;
    void ResetHeading();

    void ResetPose(frc::Pose2d pose);
    frc::Pose2d GetPose() const;

    std::array<frc::SwerveModulePosition, 4> GetModulePositions() const;
    frc::ChassisSpeeds GetRobotRelativeSpeeds() const;

private:
    SwerveDriveConfig m_config;
    std::shared_ptr<IGyroSource> m_gyro;
    std::array<SwerveModule, 4> m_modules;
    frc::SwerveDriveKinematics<4> m_kinematics;
    frc::SwerveDriveOdometry<4> m_odometry;

    std::shared_ptr<IMutex> m_commandMutex;
    frc::ChassisSpeeds m_desiredSpeeds;  // the notice board: latest posted Drive() value
};

}  // namespace NKSwerve
