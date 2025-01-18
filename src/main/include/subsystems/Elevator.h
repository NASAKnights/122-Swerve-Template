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

static constexpr units::meter_t                     upperLimit    = 72_in;
static constexpr units::meter_t                     lowerLimit    = 0_in;
static constexpr units::meter_t                     simUpperLimit = 72.1_in;
static constexpr units::meter_t                     simLowerLimit = -0.1_in;
static constexpr units::meters_per_second_t         kMaxVelocity  = 61.55_in / 1_s;
static constexpr units::meters_per_second_squared_t kMaxAcceleration =
    460_in / (1_s * 1_s);                 // 12.0_V * 2.0_in / (1_s * 1_s * 1_V);
static constexpr double        kP = 15.0; // 0.6
static constexpr double        kI = 0.0;  // 0.0
static constexpr double        kD = 0.0;
static constexpr units::volt_t kS = 0.2_V; // minimum voltage to move motor

static constexpr units::meter_t             kTolerancePos = 0.01_m;
static constexpr units::meters_per_second_t kToleranceVel = 0.05_mps;

const int kMotorId            = 6;
const int kEncoderPulsePerRev = 42;

static constexpr auto kFFks = 0.23_V;                 // Volts static (motor)
static constexpr auto kFFkg = 0.0_V;                  // Volts
static constexpr auto kFFkV = 0.5 * 2.32_V / 1.0_mps; // volts*s/meters //1.01
static constexpr auto kFFkA = 0.05_V / 1.0_mps_sq;    // volts*s^2/meters //0.1

static constexpr units::second_t kDt = 20_ms;

// number of motors
static constexpr int kNumMotors = 2;
// gearing between motor and drum
static constexpr double kElevatorGearing = 5;
// effective carriage mass: carriage mass = m
// maths: 1 stage = 1/1 * m1, 2 stage = 1/2 * m1 + 2/2 * m2, 3 stage = 1/3 * m1 + 2/3 * m2 + 3/3 *
// m3 highest number stage = carriage
static constexpr units::kilogram_t kCarriageMass = (17.5_lb + 0.5 * 5_lb);
// effective drum radius = radius of first stage * number of stages
static constexpr units::meter_t kElevatorDrumRadius = 1.432_in * 2;
}

class ElevatorSubsystem : public frc2::ProfiledPIDSubsystem<units::meter>
{
    using State = frc::TrapezoidProfile<units::meter>::State;

public:
    ElevatorSubsystem();
    void printLog();
    // void           handle_Setpoint();
    void           Emergency_Stop();
    void           SimulationInit();
    void           SimulationPeriodic();
    double         GetHeight();
    void           UseOutput(double output, State setpoint) override;
    units::meter_t GetMeasurement() override;
    void           HoldPosition();
    /*
    void           SetSpeed(double speed);
    */
    void SetHeight(double height);
    // bool           CheckGoal();
    void Periodic();
    // void SetSpeed(double speed);

private:
    rev::CANSparkFlex         m_motor;
    frc::ElevatorFeedforward  m_feedforwardElevator;
    wpi::log::DoubleLogEntry  m_HeightLog;
    wpi::log::DoubleLogEntry  m_SetPointLog;
    wpi::log::IntegerLogEntry m_StateLog;
    wpi::log::DoubleLogEntry  m_MotorCurrentLog;
    wpi::log::DoubleLogEntry  m_MotorVoltageLog;
    rev::SparkRelativeEncoder m_encoder;

    frc::Timer m_simTimer;

    frc::sim::ElevatorSim m_elevatorSim;

    double m_holdHeight;
    // double numStages;
    // double numMotors;
    //  units::meter_t elevatorDrumRadius;

    ElevatorConstants::ElevatorState m_ElevatorState;
};