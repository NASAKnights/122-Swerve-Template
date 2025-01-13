#pragma once

#include "frc/DataLogManager.h"
#include "wpi/DataLog.h"
#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/DutyCycleEncoder.h>
#include <frc/Encoder.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/system/plant/DCMotor.h>
#include <frc2/command/ProfiledPIDSubsystem.h>
#include <rev/CANSparkFlex.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/mass.h>
#include <units/time.h>

#include "Constants.hpp"
#include <frc/DigitalInput.h>
#include <frc/RobotBase.h>
#include <frc/Servo.h>
#include <frc/Timer.h>
#include <frc/simulation/ElevatorSim.h>
#include <frc/simulation/SimDeviceSim.h>

namespace ElevatorConstants
{

enum ElevatorState
{
    LIFT,
    LOWER,
    HOLD,
    MANUAL
};

const auto                                          upperLimit       = 3_m;
const auto                                          lowerLimit       = 0_m;
const auto                                          simUpperLimit    = 3.05_m;
const auto                                          simLowerLimit    = -0.05_m;
static constexpr units::meters_per_second_t         kMaxVelocity     = 2.0_mps;
static constexpr units::meters_per_second_squared_t kMaxAcceleration = 1.0_mps_sq;
static constexpr double                             kP               = 100.0; // 0.6
static constexpr double                             kI               = 50.0;  // 0.0
static constexpr double                             kD               = 0.0;
static constexpr units::volt_t                      kS = 0.2_V; // minimum voltage to move motor

static constexpr units::meter_t             kTolerancePos = 0.01_m;
static constexpr units::meters_per_second_t kToleranceVel = 0.05_mps;

const int kMotorId            = 6;
const int kEncoderPulsePerRev = 42;

static constexpr auto kFFks = 0.23_V;             // Volts static (motor)
static constexpr auto kFFkg = 0.0_V;              // Volts
static constexpr auto kFFkV = 3.0_V / 1.0_mps;    // volts*s/meters //1.01
static constexpr auto kFFkA = 1.0_V / 1.0_mps_sq; // volts*s^2/meters //0.1

static constexpr units::second_t kDt = 20_ms;

static constexpr double kElevatorGearing    = 10;
static constexpr auto   kCarriageMass       = 5_kg;
static constexpr auto   kElevatorDrumRadius = 0.1_m;
}

class ElevatorSubsystem : public frc2::ProfiledPIDSubsystem<units::meter>
{
    using State = frc::TrapezoidProfile<units::meter>::State;

public:
    ElevatorSubsystem();
    void           printLog();
    void           handle_Setpoint();
    void           Emergency_Stop();
    void           SimulationPeriodic();
    double         GetHeight();
    void           UseOutput(double output, State setpoint) override;
    units::meter_t GetMeasurement() override;
    void           HoldPosition();
    /*
    void           SetSpeed(double speed);
    */
    void SetHeight(double height);
    /*
    bool           CheckGoal();
    void           Periodic();
    */

private:
    rev::CANSparkFlex         m_motor;
    frc::ElevatorFeedforward  m_feedforwardElevator;
    wpi::log::DoubleLogEntry  m_HeightLog;
    wpi::log::DoubleLogEntry  m_SetPointLog;
    wpi::log::IntegerLogEntry m_StateLog;
    wpi::log::DoubleLogEntry  m_MotorCurrentLog;
    wpi::log::DoubleLogEntry  m_MotorVoltageLog;
    frc::Timer*               m_timer;
    rev::SparkRelativeEncoder m_encoder;

    frc::Timer m_simTimer;

    frc::sim::ElevatorSim m_elevatorSim;

    double m_holdHeight;

    ElevatorConstants::ElevatorState m_ElevatorState;
};