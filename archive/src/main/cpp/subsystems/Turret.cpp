// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "subsystems/Turret.h"

#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Translation2d.h>
#include <cmath>

using State = frc::TrapezoidProfile<units::degrees>::State;
using degrees_per_second_squared_t =
    units::unit_t<units::compound_unit<units::angular_velocity::degrees_per_second,
                                       units::inverse<units::time::seconds>>>;

Turret::Turret() : m_controller(
                       TurretConstants::kAngleP, TurretConstants::kAngleI, TurretConstants::kAngleD,
                       frc::TrapezoidProfile<units::degrees>::Constraints(TurretConstants::kTurretVelLimit, TurretConstants::kTurretAccelLimit), 5_ms),

                   m_motor(TurretConstants::kAngleMotorId), m_feedforward(TurretConstants::kFFks, TurretConstants::kFFkg, TurretConstants::kFFkV,
                                                                          TurretConstants::kFFkA),

                   m_TurretSim(TurretConstants::kSimMotor, TurretConstants::kGearRatio, TurretConstants::kmoi,
                               TurretConstants::kTurretRadius, TurretConstants::kminAngle, TurretConstants::kmaxAngle,
                               TurretConstants::kGravity, TurretConstants::kTurretStartAngle, TurretConstants::kSimNoise)
{
    m_motor.SetInverted(true);
    m_controller.SetIZone(TurretConstants::kIZone);

    m_controller.SetTolerance(TurretConstants::kTolerancePos, TurretConstants::kToleranceVel);
    // Start m_Turret in neutral position
    m_TurretState = TurretConstants::TRACKING;
    wpi::log::DataLog &log = frc::DataLogManager::GetLog();
    m_AngleLog = wpi::log::DoubleLogEntry(log, "/Turret/Angle");
    m_SetPointLog = wpi::log::DoubleLogEntry(log, "/Turret/Setpoint");
    m_StateLog = wpi::log::IntegerLogEntry(log, "/Turret/State");
    m_MotorCurrentLog = wpi::log::DoubleLogEntry(log, "/Turret/MotorCurrent");
    m_MotorVoltageLog = wpi::log::DoubleLogEntry(log, "/Turret/MotorVoltage");

    networkTableInst = nt::NetworkTableInstance::GetDefault();
    auto poseTable = networkTableInst.GetTable("ROS2Bridge");
    baseLinkSubscriber = poseTable->GetDoubleArrayTopic(robotPoseLink).Subscribe({}, {.periodic = 0.02, .sendAll = true});

    m_turretObject = m_turretField.GetObject("Turret");
    frc::SmartDashboard::PutData("Turret Field", &m_turretField);
    m_motor.SetPosition(0.0_rad);
    SetAngle(0.0_deg);

    // if constexpr(frc::RobotBase::IsSimulation())
    // {
    //     m_simTimer.Start();
    // }
    // const frc::DCMotor, const double, const units::moment_of_inertia::kilogram_square_meter_t, const units::length::meter_t,
    // const units::angle::radian_t, const units::angle::radian_t, const bool, const units::angle::radian_t, const std::array<double, 1U>)
}

void Turret::SimulationPeriodic()
{
    m_TurretSim.Update(10_ms);
    frc::SmartDashboard::PutNumber("Motor current draw", m_TurretSim.GetCurrentDraw().value());
}

units::degree_t Turret::GetMeasurement()
{ // original get measurement function
    if constexpr (frc::RobotBase::IsSimulation())
    {
        return m_TurretSim.GetAngle();
    }

    return units::turn_t{(m_motor.GetPosition().GetValue() / TurretConstants::kGearRatio)};
}

units::degree_t Turret::findTrackingAngle()
{
    std::vector<double> baseLinkPose = baseLinkSubscriber.Get({});
    auto baseLink = DoubleArrayToPose2d(baseLinkPose);
    if (!baseLink.has_value())
    {
        return GetMeasurement();
    }

    frc::Transform3d world2robot = frc::Transform3d(baseLink->X(), baseLink->Y(), 0_m, frc::Rotation3d(0_rad, 0_rad, baseLink->Rotation().Radians()));

    // goalSubscriber = poseTable->GetDoubleArrayTopic(goalPoseLink).Subscribe({}, {.periodic = 0.01, .sendAll = true});

    // std::vector<double> goalPose = goalSubscriber.GetAtomic().value;
    // auto world2goal = DoubleArrayToPose2d(goalPose);

    frc::Transform3d world2goal = goal;

    // world2turret rotation matrix
    frc::Transform3d world2turret =
        frc::Transform3d(baseLink->X() + units::length::meter_t{TurretConstants::kXOffset},
                         baseLink->Y() + units::length::meter_t{TurretConstants::kYOffset},
                         units::length::meter_t{TurretConstants::kZOffset},
                         frc::Rotation3d(0.0_rad, 0.0_rad,
                                         units::angle::radian_t{GetMeasurement().convert<units::angle::radians>()} + baseLink->Rotation().Radians()));

    // grab world2robot, world2goal, robot2turret transforms

    // get robot2turret transform from world2robot * (world2turret)^-1

    frc::Transform3d robot2turret = frc::Transform3d(world2robot.ToMatrix().inverse() * world2turret.ToMatrix());
    // get turret2goal transform from (world2turret)^-1 * world2goal

    // Compute world-space vector from turret to goal
    Eigen::Vector3d tgVector = (world2goal.Translation() - world2turret.Translation()).ToVector();

    // Compute desired yaw in world frame
    units::radian_t targetYaw = units::radian_t{std::atan2(tgVector.y(), tgVector.x())};

    // Get current turret yaw in world frame
    units::radian_t turretYaw = world2turret.Rotation().ToRotation2d().Radians();

    // Find smallest signed error
    units::radian_t error = frc::AngleModulus(targetYaw - turretYaw);

    // Add to current turret angle
    units::degree_t newTarget = GetMeasurement() + units::degree_t{error};

    // If the computed target exceeds the upper limit by >180°, it likely wrapped
    if (newTarget > TurretConstants::kmaxAngle)
    {
        // If we’re only just beyond by less than 180°, clamp
        if (newTarget - 360_deg >= TurretConstants::kminAngle)
            newTarget -= 360_deg;
        else
            newTarget = TurretConstants::kmaxAngle;
    }
    else if (newTarget < TurretConstants::kminAngle)
    {
        if (newTarget + 360_deg <= TurretConstants::kmaxAngle)
            newTarget += 360_deg;
        else
            newTarget = TurretConstants::kminAngle;
    }
    frc::SmartDashboard::PutNumber("/Turret/newTarget", double(newTarget));
    return newTarget;
}

void Turret::SetAngle(units::degree_t TurretAngleGoal)
{
    if (!(TurretAngleGoal.value() < m_goal.value() + TurretConstants::kTolerancePos.value() && TurretAngleGoal.value() > m_goal.value() - TurretConstants::kTolerancePos.value()))
    {
        if ((TurretAngleGoal <= TurretConstants::kmaxAngle) &&
            (TurretAngleGoal >= TurretConstants::kminAngle))
        {
            auto velocity = GetVelocity();
            m_goal = units::angle::degree_t(TurretAngleGoal);
            if (abs(velocity.value()) < (1_deg_per_s).value())
            {
                velocity = 1_deg_per_s * copysign(1.0, velocity.value());
            }
            m_controller.Reset(GetMeasurement(), GetVelocity());
            m_controller.SetGoal(m_goal);
        }
    }
    frc::SmartDashboard::PutNumber("/Turret/m_goal", double(m_goal));
}

units::degrees_per_second_t Turret::GetVelocity()
{
    return m_motor.GetVelocity().GetValue() / TurretConstants::kGearRatio;
}

void Turret::Periodic()
{

    printLog();
    UpdateFieldVisuals();
    double fb;
    units::volt_t ff;
    units::volt_t v;
    switch (m_TurretState)
    {
    case TurretConstants::HOLD:
    {
        // if (isTracking)
        // {
        //     m_TurretState = TurretConstants::TRACKING;
        // }
        frc::SmartDashboard::PutString("/Turret/State", "HOLD");
        fb = m_controller.Calculate(GetMeasurement());
        ff = m_feedforward.Calculate(units::radian_t{m_controller.GetSetpoint().position}, units::radians_per_second_t{m_controller.GetSetpoint().velocity}, units::radians_per_second_squared_t{m_controller.GetSetpoint().velocity / 1_s});
        v = units::volt_t{fb} + ff;
        break;
    }
    case TurretConstants::DISABLED:
    {
        frc::SmartDashboard::PutString("/Turret/State", "DISABLED");
        v = units::voltage::volt_t(0.0);
        break;
    }
    case TurretConstants::TRACKING:
    {
        frc::SmartDashboard::PutString("/Turret/State", "TRACKING");
        SetAngle(findTrackingAngle());
        fb = m_controller.Calculate(GetMeasurement());
        ff = m_feedforward.Calculate(units::radian_t{m_controller.GetSetpoint().position}, units::radians_per_second_t{m_controller.GetSetpoint().velocity}, units::radians_per_second_squared_t{m_controller.GetSetpoint().velocity / 1_s});
        v = units::volt_t{fb} + ff;
    }
    default:
    {
        frc::SmartDashboard::PutString("/Turret/State", "default");
        break;
    }
    }
    if constexpr (frc::RobotBase::IsSimulation())
    {
        m_TurretSim.SetInputVoltage(v);
        SimulationPeriodic();
    }
    m_motor.SetVoltage(v);
}

TurretConstants::TurretState Turret::GetState()
{
    return m_TurretState;
}

void Turret::printLog()
{
    frc::SmartDashboard::PutNumber("/Turret/Actual Angle", GetMeasurement().value());
    frc::SmartDashboard::PutNumber("/Turret/Goal Angle", m_controller.GetGoal().position.value());
    frc::SmartDashboard::PutNumber("/Turret/setpoint",
                                   m_controller.GetSetpoint().position.value());
    frc::SmartDashboard::PutNumber("/Turret/velocity", double(GetVelocity()));
    m_AngleLog.Append(GetMeasurement().value());
    m_SetPointLog.Append(m_controller.GetSetpoint().position.value());
    m_StateLog.Append(m_TurretState);
    // m_MotorCurrentLog.Append(m_motor.GetOutputCurrent());
    // m_MotorVoltageLog.Append(m_motor.GetAppliedOutput());
}

void Turret::Disable()
{
    m_motor.StopMotor();
}

void Turret::HoldPosition()
{
    // if (m_TurretState != TurretConstants::TurretState::HOLD)
    {
        m_controller.Reset(GetMeasurement());
        m_controller.SetGoal(GetMeasurement());
        m_goal = GetMeasurement();
        m_TurretState = TurretConstants::TurretState::HOLD;
    }
}

void Turret::UpdateFieldVisuals()
{
    m_turretField.GetObject("Goal")->SetPose(frc::Pose3d(goal.ToMatrix()).ToPose2d());
    if (m_turretObject == nullptr)
    {
        return;
    }

    auto baseLinkPose = DoubleArrayToPose2d(baseLinkSubscriber.Get({}));
    if (baseLinkPose.has_value())
    {
        m_turretField.SetRobotPose(*baseLinkPose);
        auto turretPose = CalculateTurretPose(*baseLinkPose);
        m_turretObject->SetPose(turretPose);
        m_lastRobotPose = *baseLinkPose;
        m_lastTurretPose = turretPose;
        return;
    }

    if (m_lastRobotPose.has_value())
    {
        m_turretField.SetRobotPose(*m_lastRobotPose);
    }

    if (m_lastTurretPose.has_value())
    {
        m_turretObject->SetPose(*m_lastTurretPose);
    }
    else
    {
        // Default to robot origin so the turret object stays visible even before NT data arrives.
        auto defaultPose = CalculateTurretPose(frc::Pose2d{});
        m_turretObject->SetPose(defaultPose);
        m_lastTurretPose = defaultPose;
    }
}

frc::Pose2d Turret::CalculateTurretPose(const frc::Pose2d &robotPose)
{
    // The turret's pose relative to the robot center
    frc::Transform2d turretTransform{
        frc::Translation2d{
            units::meter_t{TurretConstants::kXOffset},
            units::meter_t{TurretConstants::kYOffset}},
        frc::Rotation2d{GetMeasurement() + TurretConstants::kAngleOffset}};

    // Apply that transform in the robot's frame to get field-relative turret pose
    return robotPose.TransformBy(turretTransform);
}
