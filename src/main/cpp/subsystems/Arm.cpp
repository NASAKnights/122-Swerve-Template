// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "subsystems/Arm.h"

using State = frc::TrapezoidProfile<units::degrees>::State;
using degrees_per_second_squared_t =
    units::unit_t<units::compound_unit<units::angular_velocity::degrees_per_second,
                                       units::inverse<units::time::seconds>>>;

ArmSubsystem::ArmSubsystem()
    : frc2::ProfiledPIDSubsystem<units::degrees>(frc::ProfiledPIDController<units::degrees>(
          ArmConstants::kAngleP, ArmConstants::kAngleI, ArmConstants::kAngleD,
          frc::TrapezoidProfile<units::degrees>::Constraints(ArmConstants::kArmVelLimit,
                                                             ArmConstants::kArmAccelLimit),
          5_ms))
    , m_motor(ArmConstants::kAngleMotorId, rev::CANSparkLowLevel::MotorType::kBrushless)
    , m_feedforward(ArmConstants::kFFks, ArmConstants::kFFkg, ArmConstants::kFFkV,
                    ArmConstants::kFFkA)
    , Linear{1}
    , m_encoder{m_motor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor,
                                   ArmConstants::kAngleEncoderPulsePerRev)}
    , m_ArmSim(frc::DCMotor::NeoVortex(1), ArmConstants::kGearRatio, ArmConstants::kmoi,
               ArmConstants::kArmLength, ArmConstants::kminAngle, ArmConstants::kmaxAngle,
               ArmConstants::kGravity, ArmConstants::kArmStartAngle, ArmConstants::kSimNoise)
{
    GetController().SetIZone(ArmConstants::kIZone);
    m_encoder.SetPositionConversionFactor(6.0);

    m_timer = new frc::Timer();
    m_motor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);

    GetController().SetTolerance(ArmConstants::kControllerTolerance);
    // Start m_arm in neutral position
    SetGoal(State{units::degree_t(ArmConstants::kArmAngleRetracted), 0_rad_per_s});

    m_ArmState = ArmConstants::ArmState::BACKWARD;

    wpi::log::DataLog& log = frc::DataLogManager::GetLog();
    m_AngleLog             = wpi::log::DoubleLogEntry(log, "/Arm/Angle");
    m_SetPointLog          = wpi::log::DoubleLogEntry(log, "/Arm/Setpoint");
    m_StateLog             = wpi::log::IntegerLogEntry(log, "/Arm/State");
    m_MotorCurrentLog      = wpi::log::DoubleLogEntry(log, "/Arm/MotorCurrent");
    m_MotorVoltageLog      = wpi::log::DoubleLogEntry(log, "/Arm/MotorVoltage");

    // if constexpr(frc::RobotBase::IsSimulation())
    // {
    //     m_simTimer.Start();
    // }
}

void ArmSubsystem::SimulationPeriodic()
{
    m_ArmSim.Update(5_ms);
    frc::SmartDashboard::PutNumber("Motor current draw", m_ArmSim.GetCurrentDraw().value());
}
void ArmSubsystem::UseOutput(double output, State setpoint)
{
    // Calculate the feedforward from the sepoint
    units::volt_t feedforward = m_feedforward.Calculate(setpoint.position, setpoint.velocity);
    if constexpr(frc::RobotBase::IsSimulation())
    {
        m_ArmSim.SetInputVoltage(units::volt_t{output} + feedforward);
    }
    m_motor.SetVoltage(units::volt_t{output} + feedforward);
}

void ArmSubsystem::Emergency_Stop()
{
    if(Kill.Get() == true)
    {
        m_motor.StopMotor();
    }
}

bool ArmSubsystem::isOverLimit()
{
    return Kill.Get();
}

void ArmSubsystem::printLog()
{
    frc::SmartDashboard::PutNumber("ARM_ENC_ABS", GetMeasurement().value());
    frc::SmartDashboard::PutNumber("armGoal_POS", GetController().GetGoal().position.value());
    frc::SmartDashboard::PutNumber("ARM_setpoint", GetController().GetSetpoint().position.value());
    m_AngleLog.Append(GetMeasurement().value());
    m_SetPointLog.Append(GetController().GetSetpoint().position.value());
    m_StateLog.Append(m_ArmState);
    m_MotorCurrentLog.Append(m_motor.GetOutputCurrent());
    m_MotorVoltageLog.Append(m_motor.GetAppliedOutput());
}

units::degree_t ArmSubsystem::GetMeasurement()
{ // original get measurement function
    if constexpr(frc::RobotBase::IsSimulation())
    {
        return m_ArmSim.GetAngle();
    }

    return units::degree_t{m_encoder.GetPosition()};
}

void ArmSubsystem::kick()
{
    if(ArmConstants::PRIMED == m_ArmState)
    {
        ArmSubsystem::m_ArmState = ArmConstants::STARTKICK;
    }
}
void ArmSubsystem::handle_Setpoint()
{
    frc::SmartDashboard::PutNumber("Arm Goal Angle", GetController().GetGoal().position.value());
    frc::SmartDashboard::PutNumber("Arm Actual Angle", GetMeasurement().value());
    switch(m_ArmState)
    {
        case ArmConstants::STARTKICK:
        {
            SetGoal(units::angle::degree_t{ArmConstants::kArmAngleExtended});
            m_ArmState = ArmConstants::FORWARD;
            m_timer->Restart();
            frc::SmartDashboard::PutString("State", "StartKick");
        }
        case ArmConstants::FORWARD:
        {
            // Forward state is set when Kick is in proccess
            //  After full kicking motion has been completed (successful angle has been reach)
            //  Return to backwards position
            frc::SmartDashboard::PutString("State", "Forward");
            if(GetController().AtGoal() || m_timer->Get().value() > ArmConstants::kMaxTimer)
            {
                m_ArmState = ArmConstants::BACKWARD;
                SetGoal(units::angle::degree_t{ArmConstants::kArmAngleRetracted});
            }
            break;
        }
        case ArmConstants::PRIMED:
        {
            // Driver input Causes kicking motion going from Primed to Extended
            frc::SmartDashboard::PutString("State", "Primed");
            break;
        }
        case ArmConstants::BACKWARD:
        {
            // Return Arm to backwards set position
            frc::SmartDashboard::PutString("State", "Backward");
            if(GetController().AtGoal())
            {
                m_ArmState = ArmConstants::PRIMED;
                SetGoal(units::angle::degree_t{ArmConstants::kArmAngleExtended});
            }
            break;
        }
        default:
            frc::SmartDashboard::PutString("State", "Default");
            break;
    }
}
