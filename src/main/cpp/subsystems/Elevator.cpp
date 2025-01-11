/*
https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/physics-sim.html
https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robotbuilder/advanced/robotbuilder-writing-pidsubsystem-code.html
https://github.wpilib.org/allwpilib/docs/release/cpp/classfrc_1_1sim_1_1_elevator_sim.html
https://github.com/wpilibsuite/allwpilib/blob/main/sysid/src/main/native/cpp/analysis/ElevatorSim.cpp
*/

#include "subsystems/Elevator.h"

using State = frc::TrapezoidProfile<units::meters>::State;
using mps_sq_t =
    units::unit_t<units::compound_unit<units::velocity::mps, units::inverse<units::time::seconds>>>;

ElevatorSubsystem::ElevatorSubsystem()
    : frc2::ProfiledPIDSubsystem<units::meter>(frc::ProfiledPIDController<units::meter>(
          ElevatorConstants::kP, ElevatorConstants::kI, ElevatorConstants::kD,
          frc::TrapezoidProfile<units::meter>::Constraints(ElevatorConstants::kMaxVelocity,
                                                           ElevatorConstants::kMaxAcceleration),
          5_ms))
    , m_motor(ElevatorConstants::kMotorId, rev::CANSparkLowLevel::MotorType::kBrushless)
    , m_encoder{m_motor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor,
                                   ElevatorConstants::kEncoderPulsePerRev)}
    , m_feedforwardElevator{ElevatorConstants::kFFks, ElevatorConstants::kFFkg,
                            ElevatorConstants::kFFkV, ElevatorConstants::kFFkA}
    , m_elevatorSim(frc::DCMotor::NeoVortex(1), ElevatorConstants::kElevatorGearing,
                    ElevatorConstants::kCarriageMass, ElevatorConstants::kElevatorDrumRadius,
                    ElevatorConstants::simLowerLimit, ElevatorConstants::simUpperLimit, true, 0_m,
                    {0.01})
{
    wpi::log::DataLog& log = frc::DataLogManager::GetLog();
    m_HeightLog            = wpi::log::DoubleLogEntry(log, "/Elevator/Angle");
    m_SetPointLog          = wpi::log::DoubleLogEntry(log, "/Elevator/Setpoint");
    m_StateLog             = wpi::log::IntegerLogEntry(log, "/Elevator/State");
    m_MotorCurrentLog      = wpi::log::DoubleLogEntry(log, "/Elevator/MotorCurrent");
    m_MotorVoltageLog      = wpi::log::DoubleLogEntry(log, "/Elevator/MotorVoltage");

    // m_encoder.SetDistancePerPulse(0.0);
    //  m_encoder.Reset();
    m_holdHeight    = 0.0;
    m_ElevatorState = ElevatorConstants::ElevatorState::HOLD;
}

void ElevatorSubsystem::HoldPosition()
{
    m_controller.SetTolerance(ElevatorConstants::kTolerancePos, ElevatorConstants::kToleranceVel);
    m_controller.SetGoal(units::meter_t{GetHeight()});
    m_ElevatorState = ElevatorConstants::ElevatorState::HOLD;
}

void ElevatorSubsystem::SetHeight(double height)
{
    if(m_ElevatorState != ElevatorConstants::ElevatorState::LIFT &&
       m_ElevatorState != ElevatorConstants::ElevatorState::LOWER)
    {
        if(GetHeight() > height)
        {
            m_ElevatorState = ElevatorConstants::ElevatorState::LOWER;
            m_controller.Reset(units::meter_t{GetHeight()});
            m_controller.SetTolerance(ElevatorConstants::kTolerancePos,
                                      ElevatorConstants::kToleranceVel);
            m_controller.SetGoal(units::meter_t{height});
        }
        else if(GetHeight() < height)
        {
            m_ElevatorState = ElevatorConstants::ElevatorState::LIFT;
            m_controller.Reset(units::meter_t{GetHeight()});
            m_controller.SetTolerance(ElevatorConstants::kTolerancePos,
                                      ElevatorConstants::kToleranceVel);
            m_controller.SetGoal(units::meter_t{height});
        }
        else
        {
            m_ElevatorState = ElevatorConstants::ElevatorState::HOLD;
        }
    }
}

/*
void ElevatorSubsystem::SetSpeed(double speed)
{
    if(m_ElevatorState == ElevatorConstants::ElevatorState::MANUAL)
    {
        m_motor.Set(speed);
    }
}
*/

double ElevatorSubsystem::GetHeight()
{
    if constexpr(frc::RobotBase::IsSimulation())
    {
        // frc::SmartDashboard::PutBoolean("isSim", true);
        return m_elevatorSim.GetPosition().value();
    }
    return m_encoder.GetPosition();
}

units::meter_t ElevatorSubsystem::GetMeasurement()
{
    return units::meter_t{GetHeight()};
}
/*
bool ElevatorSubsystem::CheckGoal()
{
    return m_ElevatorState == ElevatorConstants::ElevatorState::HOLD;
}
*/
void ElevatorSubsystem::printLog()
{
    frc::SmartDashboard::PutNumber("ELEVATOR_ENC_ABS", GetMeasurement().value());
    frc::SmartDashboard::PutNumber("elevatorGoal_POS", GetController().GetGoal().position.value());
    frc::SmartDashboard::PutNumber("ELEVATOR_setpoint",
                                   GetController().GetSetpoint().position.value());
    m_HeightLog.Append(GetMeasurement().value());
    m_SetPointLog.Append(GetController().GetSetpoint().position.value());
    m_StateLog.Append(m_ElevatorState);
    m_MotorCurrentLog.Append(m_motor.GetOutputCurrent());
    m_MotorVoltageLog.Append(m_motor.GetAppliedOutput());
}
void ElevatorSubsystem::handle_Setpoint()
{
    // check if at goal
    if(m_ElevatorState != ElevatorConstants::ElevatorState::HOLD && m_controller.AtGoal())
    {
        m_ElevatorState = ElevatorConstants::ElevatorState::HOLD;
    }
    frc::SmartDashboard::PutNumber("Elevator Goal Height",
                                   GetController().GetGoal().position.value());
    frc::SmartDashboard::PutNumber("Elevator Actual Height", GetMeasurement().value());
    // This method will be called once per scheduler run.
    switch(m_ElevatorState)
    {
        case ElevatorConstants::LIFT:
            frc::SmartDashboard::PutString("ElevState", "LIFT");
            break;
        case ElevatorConstants::LOWER:
            frc::SmartDashboard::PutString("ElevState", "LOWER");
            break;
        case ElevatorConstants::MANUAL:
            frc::SmartDashboard::PutString("ElevState", "MANUAL");
            break;
        case ElevatorConstants::HOLD:
            frc::SmartDashboard::PutString("ElevState", "HOLD");
            break;
    }
    printLog();
}

void ElevatorSubsystem::Emergency_Stop() {}

void ElevatorSubsystem::SimulationPeriodic()
{
    m_elevatorSim.Update(5_ms);
}
void ElevatorSubsystem::UseOutput(double output, State setpoint)
{
    // Calculate the feedforward from the sepoint
    units::volt_t feedforward = m_feedforwardElevator.Calculate(setpoint.velocity);
    units::volt_t v           = units::volt_t{output} + feedforward;
    if constexpr(frc::RobotBase::IsSimulation())
    {
        m_elevatorSim.SetInputVoltage(v);
    }
    m_motor.SetVoltage(v);
    frc::SmartDashboard::PutNumber("Elev_UO_PID", output);
    frc::SmartDashboard::PutNumber("Elev_UO_FF", feedforward.value());
    frc::SmartDashboard::PutNumber("Elev_UO_Volt", v.value());
}
