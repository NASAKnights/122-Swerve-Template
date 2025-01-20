#pragma once

#include "frc/DataLogManager.h"
#include "wpi/DataLog.h"
#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/DutyCycleEncoder.h>
#include <frc/Encoder.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/ProfiledPIDSubsystem.h>
#include <rev/CANSparkFlex.h>
#include <units/angle.h>
#include <units/time.h>

#include "Constants.hpp"
#include <frc/DigitalInput.h>
#include <frc/RobotBase.h>
#include <frc/Servo.h>
#include <frc/Timer.h>
#include <frc/simulation/SimDeviceSim.h>
#include <frc/simulation/SingleJointedArmSim.h>

namespace ArmConstants
{

enum ArmState
{
    FORWARD,
    BACKWARD,
    PRIMED,
    STARTKICK
    // UNBRAKED,
};

const double kAngleP        = 0.6;
const double kAngleI        = 0.0;
const double kAngleD        = 0.0; // 0.0001
const double kIZone         = 1.0;
const auto   kArmVelLimit   = units::degrees_per_second_t(4480.0);
const auto   kArmAccelLimit = units::angular_acceleration::degrees_per_second_squared_t(
    48000.0); // Mech limit 27 rad/s^2(1500 degree_second_squared)
const auto kControllerTolerance = units::degree_t(1.0);
const int  kAngleMotorId        = 5;

const int  kAngleEncoderPulsePerRev = 42;
const auto kFFks                    = units::volt_t(0.23);            // Volts static (motor)
const auto kFFkg                    = units::volt_t(0.28);            // Volts
const auto kFFkV = units::unit_t<frc::ArmFeedforward::kv_unit>(1.01); // volts*s/rad
const auto kFFkA = units::unit_t<frc::ArmFeedforward::ka_unit>(0.01); // volts*s^2/rad

const bool   kArmEnableCurrentLimit     = true;
const int    kArmContinuousCurrentLimit = 35;
const int    kArmPeakCurrentLimit       = 60;
const double kArmPeakCurrentDuration    = 0.1;

const std::array<double, 1> kSimNoise = {0.0087};

const double kArmAngleStarting  = 0.0;   // With offset
const double kArmAngleRetracted = 45.0;  // With offset
const double kArmAngleExtended  = -40.0; // with offset
const double kMaxTimer          = 2.0;   // Max time for kicking (seconds)

const double kGearRatio = 60.0; // gear ratio for motor to arm
const units::moment_of_inertia::kilogram_square_meter_t kmoi =
    units::moment_of_inertia::kilogram_square_meter_t(0.175);
const units::length::meter_t kArmLength     = units::length::meter_t(0.4064);
const units::angle::radian_t kminAngle      = units::angle::radian_t(-0.785);
const units::angle::radian_t kmaxAngle      = units::angle::radian_t(1.571);
const bool                   kGravity       = true;
const units::angle::radian_t kArmStartAngle = units::angle::radian_t(0.0);

} // namespace ArmConstants

/**
 * A robot m_arm subsystem that moves with a motion profile.
 */
class ArmSubsystem : public frc2::ProfiledPIDSubsystem<units::degrees>
{
    using State = frc::TrapezoidProfile<units::degrees>::State;

public:
    ArmSubsystem();
    void printLog();
    void handle_Setpoint();
    void Emergency_Stop();
    void kick();
    void SimulationPeriodic();
    // void get_pigeon();
    void            UseOutput(double output, State setpoint) override;
    units::degree_t GetMeasurement() override;
    bool            isOverLimit();

    units::time::second_t  time_brake_released;
    ArmConstants::ArmState m_ArmState;

private:
    rev::CANSparkFlex                 m_motor;
    frc::ArmFeedforward               m_feedforward;
    wpi::log::DoubleLogEntry          m_AngleLog;
    wpi::log::DoubleLogEntry          m_SetPointLog;
    wpi::log::IntegerLogEntry         m_StateLog;
    wpi::log::DoubleLogEntry          m_MotorCurrentLog;
    wpi::log::DoubleLogEntry          m_MotorVoltageLog;
    ctre::phoenix6::hardware::Pigeon2 arm_pigeon{9}; //, "NKCANivore"};
    frc::Timer*                       m_timer;
    rev::SparkRelativeEncoder         m_encoder;
    float                             ARM_Angle;
    frc::PWM                          Linear;
    frc::DigitalInput                 Kill{4};

    frc::Timer m_simTimer;

    frc::sim::SingleJointedArmSim m_ArmSim;

    hal::SimDouble m_armSimVelocity;
    hal::SimDouble m_armSimposition;
};