#pragma once

#include "frc/DataLogManager.h"
#include "wpi/DataLog.h"
#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <frc/DutyCycleEncoder.h>
#include <frc/Encoder.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/smartdashboard/FieldObject2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Quaternion.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/ProfiledPIDSubsystem.h>
#include <rev/SparkMax.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/time.h>

#include <optional>
#include <string_view>
#include <vector>

#include "Constants.hpp"
#include <frc/DigitalInput.h>
#include <frc/RobotBase.h>
#include <frc/Servo.h>
#include <frc/Timer.h>
#include <frc/simulation/SimDeviceSim.h>
#include <frc/simulation/SingleJointedArmSim.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/smartdashboard/Field2d.h>

namespace TurretConstants
{
  enum TurretState
  {
    MOVE,
    HOLD,
    START,
    TRACKING,
    DISABLED
  };

  const double kAngleP = 0.2;
  const double kAngleI = 0.01;
  const double kAngleD = 0.0; // 0.0001
  const double kIZone = 1.0;
  const auto kTurretVelLimit = units::degrees_per_second_t(200.0);
  const auto kTurretAccelLimit = units::angular_acceleration::degrees_per_second_squared_t(200); // Mech limit 27 rad/s^2(1500 degree_second_squared)
  const units::degree_t kTolerancePos = 1_deg;
  const units::degrees_per_second_t kToleranceVel = 0.5_deg_per_s;
  const int kAngleMotorId = 50;

  const auto kFFks = units::volt_t(0.23);                                // Volts static (motor)
  const auto kFFkg = units::volt_t(0.0);                                 // Volts
  const auto kFFkV = units::unit_t<frc::ArmFeedforward::kv_unit>(0.3);   // volts*s/rad
  const auto kFFkA = units::unit_t<frc::ArmFeedforward::ka_unit>(0.001); // volts*s^2/rad

  const bool kTurretEnableCurrentLimit = true;
  const int kTurretContinuousCurrentLimit = 35;
  const int kTurretPeakCurrentLimit = 60;
  const double kTurretPeakCurrentDuration = 0.1;

  const std::array<double, 2> kSimNoise = {0.0};
  const frc::DCMotor kSimMotor = frc::DCMotor::KrakenX60(1);
  const double kGearRatio = 43.0; // gear ratio for motor to arm
  const units::moment_of_inertia::kilogram_square_meter_t kmoi =
      units::moment_of_inertia::kilogram_square_meter_t(0.06742); // I = MR^2
  const units::length::meter_t kTurretRadius = units::length::meter_t(0.3048);
  const units::mass::kilogram_t kTurretMass = units::mass::kilogram_t(0.725748);
  const units::angle::radian_t kminAngle = -225_deg;
  const units::angle::radian_t kmaxAngle = 45_deg;
  const bool kGravity = false;
  const units::angle::radian_t kTurretStartAngle = units::angle::radian_t(0.0);
  const double kXOffset = 0.0;
  const double kYOffset = 0.0;
  const double kZOffset = 0.0;
  const units::degree_t kAngleOffset(0.0);

} // namespace ArmConstants

/**
 * A robot m_arm subsystem that moves with a motion profile.
 */
class Turret : public frc2::SubsystemBase
{
  using State = frc::TrapezoidProfile<units::degrees>::State;

public:
  Turret();
  void Periodic();
  void Emergency_Stop();
  void ChangeAngle();
  void UseOutput();
  void SimulationPeriodic();
  void Enable();
  void Disable();
  void SetAngle(units::degree_t angle);
  void Zero();
  void HoldPosition();
  void Reset()
  {
    m_controller.Reset(GetMeasurement(), GetVelocity());
  }
  // void get_pigeon();
  units::degree_t GetMeasurement();
  units::degrees_per_second_t GetVelocity();
  TurretConstants::TurretState GetState();
  bool isTracking = true;

  // units::time::second_t time_brake_released;

private:
  frc::Transform3d goal = frc::Transform3d(2_m, 2_m, 0_m, frc::Rotation3d());
  static std::optional<frc::Pose2d> DoubleArrayToPose2d(const std::vector<double> &arr)
  {
    if (arr.size() < 7)
    {
      return std::nullopt;
    }

    auto x = units::length::meter_t(arr.at(0));
    auto y = units::length::meter_t(arr.at(1));

    auto o = units::angle::radian_t(
        frc::Rotation3d(frc::Quaternion(arr.at(6),
                                        arr.at(3),
                                        arr.at(4),
                                        arr.at(5)))
            .ToRotation2d()
            .Radians()
            .value());

    return frc::Pose2d(x, y, o);
  }
  void UpdateFieldVisuals();
  frc::Pose2d CalculateTurretPose(const frc::Pose2d &robotPose);
  TurretConstants::TurretState m_TurretState;
  void printLog();
  ctre::phoenix6::hardware::TalonFX m_motor;
  frc::ArmFeedforward m_feedforward;
  wpi::log::DoubleLogEntry m_AngleLog;
  wpi::log::DoubleLogEntry m_SetPointLog;
  wpi::log::IntegerLogEntry m_StateLog;
  wpi::log::DoubleLogEntry m_MotorCurrentLog;
  wpi::log::DoubleLogEntry m_MotorVoltageLog;
  frc::Timer *m_timer;
  float Turret_Angle;
  units::degree_t findTrackingAngle();

  units::degree_t m_goal;
  frc::Timer m_simTimer;

  frc::sim::SingleJointedArmSim m_TurretSim;

  frc::ProfiledPIDController<units::degrees> m_controller;

  hal::SimDouble m_TurretSimVelocity;
  hal::SimDouble m_TurretSimposition;
  nt::DoubleArraySubscriber baseLinkSubscriber;
  nt::DoubleArraySubscriber goalSubscriber;
  std::string_view robotPoseLink = "base_link";
  std::string_view goalPoseLink = "goal";
  std::vector<frc::Pose2d> poses{};
  nt::NetworkTableInstance networkTableInst;
  frc::Field2d m_turretField;
  frc::FieldObject2d *m_turretObject = nullptr;
  std::optional<frc::Pose2d> m_lastRobotPose;
  std::optional<frc::Pose2d> m_lastTurretPose;
};
