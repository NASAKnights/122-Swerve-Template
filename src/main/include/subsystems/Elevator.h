#pragma once

#include "frc/DataLogManager.h"
#include "wpi/DataLog.h"
#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANdi.hpp>
#include <ctre/phoenix6/StatusSignal.hpp>
#include <frc/DutyCycleEncoder.h>
#include <frc/Encoder.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/system/plant/DCMotor.h>
#include <frc2/command/ProfiledPIDSubsystem.h>
// #include <rev/CANSparkMax.h>
#include <rev/SparkMax.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/mass.h>
#include <units/time.h>
#include <math.h>

#include "Constants.hpp"
#include <frc/DigitalInput.h>
#include <frc/RobotBase.h>
#include <frc/Servo.h>
#include <frc/Timer.h>
#include <frc/simulation/ElevatorSim.h>
#include <frc/simulation/SimDeviceSim.h>
#include <frc/controller/ProfiledPIDController.h>

namespace ElevatorConstants
{

    enum ElevatorState
    {
        DISABLED,
        START_HOLD,
        HOLDING,
        START_MOVE,
        MOVING,
        ZEROING
    };

    static constexpr units::meter_t upperLimit = 30_in; // 57
    static constexpr units::meter_t lowerLimit = 0.257_m;

    static constexpr units::meter_t simUpperLimit = 57.1_in;
    static constexpr units::meter_t simLowerLimit = -0.1_in;

    static constexpr units::meters_per_second_t kMaxVelocity = 60.0_in / 1_s;                      // 61.55
    static constexpr units::meters_per_second_squared_t kMaxAcceleration = 250.0_in / (1_s * 1_s); // 460_in / (1_s * 1_s);

    static constexpr double kP = 14.5; // 0.6 - 15

    static constexpr double kI = 0.12; // 0.0
    static constexpr double kD = 0.0;
    static constexpr units::volt_t kS = 0.2_V; // minimum voltage to move motor

    static constexpr units::meter_t kTolerancePos = 0.01_m;
    static constexpr units::meters_per_second_t kToleranceVel = 0.01_mps;

    static constexpr units::meter_t kEmergencyTolerance = 0.25_in;

    const int kMotorIdLeft = 5;
    const int kMotorIdRight = 6;
    // const int kEncoderPulsePerRev = 42;

    static constexpr auto kFFks = 0.05_V;              // Volts static (motor)
    static constexpr auto kFFkg = -0.1_V;              // 0.54_V;                         // Volts
    static constexpr auto kFFkV = 2.8_V / 1.0_mps;     // volts*s/meters //1.01 // 2.23
    static constexpr auto kFFkA = 0.35_V / 1.0_mps_sq; // volts*s^2/meters //0.1

    static constexpr units::second_t kDt = 5_ms;

    // number of motors
    static constexpr int kNumMotors = 2;
    // gearing between motor and drum
    static constexpr double kElevatorGearing = 5;
    // effective carriage mass: carriage mass = m
    // maths: 1 state = 1/1 * m1, 2 stage = 1/2 * m1 + 2/2 * m2, 3 stage = 1/3 * m1 + 2/3 * m2 + 3/3 * m3
    // highest number stage = carriage

    // static constexpr units::kilogram_t kCarriageMass = (17.5_lb + 0.5 * 5_lb);
    static constexpr units::kilogram_t kCarriageMass = (0.1_lb);
    // effective drum radius = radius of first stage * number of stages
    // pulses per rev must be set correctly in the SparkMax encoder - only reports revolutions
    // effectively, this will be the diameter because there are two stages
    static constexpr units::meter_t kElevatorDrumRadius = 1.432_in;

    // auto calibration will only occur when gong slower than this
    static constexpr units::meters_per_second_t kAutoCalMaxVelocity = 0.01_mps;

    // auto calibration will only occure when above this height
    static constexpr units::meter_t kAutoCalMinHeight = 0.05_m;

    // should be set to the expected height in meters of stage 1
    // when the cable becomes tensioned and stage 2 begins to move
    // relative to stage 1.
    // TODO need to measure this
    static const units::meter_t kInitialHeightCorrection = 0.0_m;

    // piecewise linear fit breakpoint count
    static const int kHallPwlPoints = 10;

    // piecewise linear fit for 2-magnet holders
    // piecewise linear fit breakpoint field angles in rotations (0 .. 1)
    static const double kHall2PwlAngle[kHallPwlPoints] = {
        -0.03386625, 0.00548981, 0.03846759, 0.0993575, 0.16011927,
        0.82887919, 0.90070398, 0.94538503, 1.00212295, 1.03386625};
    // piecewise linear fit breakpoint relative positions in meters
    static const double kHall2PwlPosition[kHallPwlPoints] = {
        -0.0278375, -0.02339076, -0.02053397, -0.01653541, -0.01346209,
        0.0130175, 0.01655938, 0.01931709, 0.02409609, 0.02790332};

    // piecewise linear fit for 4-magnet holders
    // piecewise linear fit breakpoint field angles in rotations (0 .. 1)
    static const double kHall4PwlAngle[kHallPwlPoints] = {
        -1.04600337, -1.00715384, -0.94230751, -0.83611974, -0.24959909,
        0.38566217, 0.8375927, 0.92915047, 0.99704411, 1.04600337};
    // piecewise linear fit breakpoint relative positions in meters
    static const double kHall4PwlPosition[kHallPwlPoints] = {
        -0.04817201, -0.0434131, -0.03794988, -0.03223475, -0.00944449,
        0.01464383, 0.03235479, 0.03713524, 0.04237394, 0.04810641};

    // number of magnet holders
    static const int kHallMagnetHolderCount = 4;
    // number of magnets in each holder
    static const int kHallMagnetCounts[kHallMagnetHolderCount] = {2, 4, 2, 2};
    // heights of the center of each holder relative to the position of the
    // sensor center when the carriage is fully retracted (corresponds to when the
    // motor encoders are zeroed).  Note that these distances are not scaled.
    // Measurement Process:
    //   - With the cable properly tensioned, put the elevator in its lowest (zeroed)
    //     position.
    //   - Measure the distance from the sensor center point to each of the
    //     L0, L1/L2, L3, and L4 mag holder centers.
    // TODO make these measurement and update values below
    static double kHallMagnetHeights[kHallMagnetHolderCount] = {0.0100, 0.1180, 0.3022, 0.7245};

    static const bool kDisableHallSensor = true;

    static double kMaxHallCalibration = 0.02;
}

class Elevator : public frc2::SubsystemBase
// class Elevator : public frc2::ProfiledPIDSubsystem<units::meter>
{
    using State = frc::TrapezoidProfile<units::meter>::State;

public:
    Elevator();
    void printLog();
    // void Emergency_Stop();
    void SimulationPeriodic();
    void SimulationInit();
    double GetHeight();
    void UseOutput(double output, State setpoint);
    units::meter_t GetMeasurement();
    void HoldPosition();
    // void           SetSpeed(double speed);
    void SetHeight(double height);
    void Periodic();
    void Disable();
    bool IsHolding();
    void Zero();
    units::meter_t GetEncoderDistance(rev::spark::SparkRelativeEncoder);
    ElevatorConstants::ElevatorState GetState()
    {
        return m_ElevatorState;
    }

private:
    double GetFusedHeight();     // meters
    double GetEncoderHeight();   // meters
    double GetEncoderVelocity(); // meters per second
    double GetHallHeight(double heightEstimate);
    double GetHallPWM();
    double GetProximityPWM();
    double GetHallPosition(double positionEstimate, int magnetCount);
    double InterpolatePWL(const double *xs, const double *ys, int count, double x);
    void AutoCalibrateHeight();

    frc::ProfiledPIDController<units::meter> m_controller;

    rev::spark::SparkMax m_motorLeft;
    rev::spark::SparkMax m_motorRight;

    frc::ElevatorFeedforward m_feedforwardElevator;

    wpi::log::DoubleLogEntry m_HeightLog;
    wpi::log::DoubleLogEntry m_SetPointLog;
    wpi::log::IntegerLogEntry m_StateLog;
    wpi::log::DoubleLogEntry m_MotorCurrentLog;
    wpi::log::DoubleLogEntry m_MotorVoltageLog;

    rev::spark::SparkRelativeEncoder m_encoderLeft;
    rev::spark::SparkRelativeEncoder m_encoderRight;

    ctre::phoenix6::hardware::CANdi m_candi;

    frc::Timer m_simTimer;
    frc::Timer *m_timer;

    frc::sim::ElevatorSim m_elevatorSim;

    ElevatorConstants::ElevatorState m_ElevatorState;

    units::meter_t m_goal;

    // raw hall pwm value, updated in Periodic
    double m_hallPwm;

    // indicates zero encoder has been detected
    // must persist if we are actually at the limit switch
    bool m_lastCalZero;

    double m_heightCorrection;
};
