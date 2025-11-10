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

Elevator::Elevator()
    : // frc2::ProfiledPIDSubsystem<units::meter>(
      m_controller(
          ElevatorConstants::kP, ElevatorConstants::kI, ElevatorConstants::kD,
          frc::TrapezoidProfile<units::meter>::Constraints(ElevatorConstants::kMaxVelocity,
                                                           ElevatorConstants::kMaxAcceleration),
          ElevatorConstants::kDt),
      m_motorLeft(ElevatorConstants::kMotorIdLeft, rev::spark::SparkMax::MotorType::kBrushless),
      m_motorRight(ElevatorConstants::kMotorIdRight, rev::spark::SparkMax::MotorType::kBrushless),
      //, m_encoder{m_motor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor,
      m_feedforwardElevator{ElevatorConstants::kFFks, ElevatorConstants::kFFkg,
                            ElevatorConstants::kFFkV, ElevatorConstants::kFFkA},
      m_encoderLeft{m_motorLeft.GetEncoder()},
      m_encoderRight{m_motorRight.GetEncoder()},
      m_candi{13}, // TODO may need canbus name
      m_elevatorSim(frc::DCMotor::NEO(ElevatorConstants::kNumMotors), ElevatorConstants::kElevatorGearing,
                    ElevatorConstants::kCarriageMass, ElevatorConstants::kElevatorDrumRadius,
                    ElevatorConstants::simLowerLimit, ElevatorConstants::simUpperLimit, false, 0_m,
                    {0.0001})
{

    m_encoderLeft.SetPosition(0.0);
    m_encoderRight.SetPosition(0.0);

    rev::spark::SparkBaseConfig m_motorRightConfig;
    rev::spark::SparkBaseConfig m_motorLeftConfig;
    m_motorRightConfig.Follow(m_motorLeft, true);
    m_motorRightConfig.SmartCurrentLimit(30);

    m_motorLeftConfig.SmartCurrentLimit(30);

    m_motorRight.Configure(m_motorRightConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);
    m_motorLeft.Configure(m_motorLeftConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);

    wpi::log::DataLog &log = frc::DataLogManager::GetLog();
    m_HeightLog = wpi::log::DoubleLogEntry(log, "/Elevator/Angle");
    m_SetPointLog = wpi::log::DoubleLogEntry(log, "/Elevator/Setpoint");
    m_StateLog = wpi::log::IntegerLogEntry(log, "/Elevator/State");
    m_MotorCurrentLog = wpi::log::DoubleLogEntry(log, "/Elevator/MotorCurrent");
    m_MotorVoltageLog = wpi::log::DoubleLogEntry(log, "/Elevator/MotorVoltage");
    m_ElevatorState = ElevatorConstants::ElevatorState::DISABLED;
    m_goal = 0.0_m;
    m_heightCorrection = ElevatorConstants::kInitialHeightCorrection.value();
}

void Elevator::HoldPosition()
{
    if (m_ElevatorState != ElevatorConstants::ElevatorState::START_HOLD &&
        m_ElevatorState != ElevatorConstants::ElevatorState::START_MOVE &&
        m_ElevatorState != ElevatorConstants::ElevatorState::HOLDING)
    {
        m_goal = units::meter_t{GetHeight()};
        m_ElevatorState = ElevatorConstants::ElevatorState::START_HOLD;
    }
}
// height in meters
void Elevator::SetHeight(double height)
{
    // frc::SmartDashboard::PutNumber("/Elevator/I_am_here", 69);
    if (m_ElevatorState != ElevatorConstants::ElevatorState::DISABLED &&
        m_ElevatorState != ElevatorConstants::ElevatorState::START_HOLD &&
        m_ElevatorState != ElevatorConstants::ElevatorState::START_MOVE)
    {
        // frc::SmartDashboard::PutNumber("/Elevator/I_am_here", 74);
        m_goal = units::meter_t{height};
        m_ElevatorState = ElevatorConstants::START_MOVE;
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

double Elevator::GetHeight()
{
    if constexpr (frc::RobotBase::IsSimulation())
    {
        // frc::SmartDashboard::PutBoolean("isSim", true);
        return m_elevatorSim.GetPosition().value();
    }
    if (ElevatorConstants::kDisableHallSensor)
        return GetEncoderHeight();
    else
        return GetFusedHeight();
}

units::meter_t Elevator::GetMeasurement()
{
    return units::meter_t{GetHeight()};
}

void Elevator::printLog()
{
    double encoderHeight = GetEncoderHeight();
    double hallHeight = GetHallHeight(encoderHeight / 2.0 - m_heightCorrection);
    frc::SmartDashboard::PutNumber("/Elevator/ELEVATOR_HEIGHT", GetMeasurement().value());
    frc::SmartDashboard::PutNumber("/Elevator/ELEVATOR_HEIGHT_FUSED", GetFusedHeight());
    frc::SmartDashboard::PutNumber("/Elevator/ELEVATOR_ENC_ABS_LEFT", m_encoderLeft.GetPosition());
    frc::SmartDashboard::PutNumber("/Elevator/ELEVATOR_ENC_ABS_RIGHT", m_encoderRight.GetPosition());
    frc::SmartDashboard::PutNumber("/Elevator/ELEVATOR_ENC_HEIGHT", encoderHeight);
    frc::SmartDashboard::PutNumber("/Elevator/ELEVATOR_HALL_PWM", m_hallPwm);
    frc::SmartDashboard::PutNumber("/Elevator/ELEVATOR_HALL_HEIGHT", hallHeight);
    frc::SmartDashboard::PutNumber("/Elevator/ELEVATOR_HEIGHT_CORRECTION", m_heightCorrection);
    frc::SmartDashboard::PutNumber("/Elevator/elevatorGoal_POS", m_controller.GetGoal().position.value());
    frc::SmartDashboard::PutNumber("/Elevator/ELEVATOR_setpoint",
                                   m_controller.GetSetpoint().position.value());
    m_HeightLog.Append(GetMeasurement().value());
    m_SetPointLog.Append(m_controller.GetSetpoint().position.value());
    m_StateLog.Append(m_ElevatorState);
    m_MotorCurrentLog.Append((m_motorLeft.GetOutputCurrent() + m_motorRight.GetOutputCurrent()) / 2.0);
    m_MotorVoltageLog.Append((m_motorLeft.GetAppliedOutput() + m_motorRight.GetAppliedOutput()) / 2.0);
}

void Elevator::Periodic()
{
    double fb;
    units::volt_t ff;
    units::volt_t v;
    if (m_motorLeft.GetReverseLimitSwitch().Get())
    {
        m_encoderLeft.SetPosition(0.0);
        m_encoderRight.SetPosition(0.0);
        if (m_ElevatorState == ElevatorConstants::ZEROING)
        {
            // m_goal = 0.1_m;
            m_ElevatorState = ElevatorConstants::HOLDING;
            SetHeight(0.1);
        }
        // m_controller.SetGoal(0.1_m);
        // m_ElevatorState = ElevatorConstants::HOLDING;
    }

    // update the hall sensor raw pwm measurement
    m_hallPwm = GetHallPWM();
    // update the proximity sensor raw pwm measurement
    double proxPwm = GetProximityPWM();
    // normalize to positive value from 0 to 1.0 in case candi is counting revolutions
    proxPwm = (proxPwm < 0 ? -1 : 1) * proxPwm;
    proxPwm -= std::trunc(proxPwm);
    frc::SmartDashboard::PutNumber("/Algae/ProximityPWM", proxPwm);

    switch (m_ElevatorState)
    {
    case ElevatorConstants::ZEROING:
        m_motorLeft.SetVoltage(units::volt_t{-0.1});
        m_motorRight.SetVoltage(units::volt_t{-0.1});
        frc::SmartDashboard::PutString("/Elevator/ElevState", "ZEROING");
        break;
    case ElevatorConstants::DISABLED:
        frc::SmartDashboard::PutString("/Elevator/ElevState", "DISABLED");
        break;
    case ElevatorConstants::START_HOLD:
        m_controller.SetTolerance(ElevatorConstants::kTolerancePos,
                                  ElevatorConstants::kToleranceVel);
        m_controller.Reset(units::meter_t{GetHeight()},
                           units::meters_per_second_t{GetEncoderVelocity()});
        m_controller.SetGoal(m_goal);
        m_ElevatorState = ElevatorConstants::HOLDING;
        frc::SmartDashboard::PutString("/Elevator/ElevState", "HOLDING");
        break;
    case ElevatorConstants::HOLDING:
        fb = m_controller.Calculate(units::meter_t{GetHeight()});
        // units::volt_t ff = m_feedforwardElevator.Calculate(m_controller.GetSetpoint().velocity);
        ff = 0_V;
        v = units::volt_t{fb} + ff;
        if constexpr (frc::RobotBase::IsSimulation())
        {
            m_elevatorSim.SetInputVoltage(v);
        }
        m_motorLeft.SetVoltage(v);
        m_motorRight.SetVoltage(v);

        AutoCalibrateHeight();

        frc::SmartDashboard::PutNumber("/Elevator/Elev_UO_PID", fb);
        frc::SmartDashboard::PutNumber("/Elevator/Elev_UO_FF", ff.value());
        frc::SmartDashboard::PutNumber("/Elevator/Elev_UO_Volt", v.value());
        frc::SmartDashboard::PutString("/Elevator/ElevState", "HOLDING");
        break;
    case ElevatorConstants::START_MOVE:
        m_controller.SetTolerance(ElevatorConstants::kTolerancePos,
                                  ElevatorConstants::kToleranceVel);
        m_controller.Reset(units::meter_t{GetHeight()},
                           units::meters_per_second_t{GetEncoderVelocity()});
        m_controller.SetGoal(m_goal);
        m_ElevatorState = ElevatorConstants::MOVING;
        frc::SmartDashboard::PutString("/Elevator/ElevState", "MOVING");
        break;
    case ElevatorConstants::MOVING:
        if (m_controller.AtGoal())
        {
            m_ElevatorState = ElevatorConstants::ElevatorState::HOLDING;
            frc::SmartDashboard::PutString("/Elevator/ElevState", "HOLDING");
        }
        else
        {
            fb = m_controller.Calculate(units::meter_t{GetHeight()});
            ff = m_feedforwardElevator.Calculate(m_controller.GetSetpoint().velocity);
            v = units::volt_t{fb} + ff;
            if constexpr (frc::RobotBase::IsSimulation())
            {
                m_elevatorSim.SetInputVoltage(v);
            }
            m_motorLeft.SetVoltage(v);
            m_motorRight.SetVoltage(v);

            frc::SmartDashboard::PutNumber("/Elevator/Elev_UO_PID", fb);
            frc::SmartDashboard::PutNumber("/Elevator/Elev_UO_FF", ff.value());
            frc::SmartDashboard::PutNumber("/Elevator/Elev_UO_Volt", v.value());
            frc::SmartDashboard::PutString("/Elevator/ElevState", "MOVING");
        }
        break;
    }

    /*
    if (abs((GetEncoderDistance(m_encoderLeft) - GetEncoderDistance(m_encoderRight)).value()) > ElevatorConstants::kEmergencyTolerance.value())
    {
        Disable();
    }
    */

    frc::SmartDashboard::PutNumber("/Elevator/Elevator Goal Height",
                                   m_goal.value());
    frc::SmartDashboard::PutNumber("/Elevator/Elevator Actual Height", GetHeight());
    // This method will be called once per scheduler run.
    printLog();
}
void Elevator::SimulationInit()
{
    m_simTimer.Reset();
}
void Elevator::SimulationPeriodic()
{
    m_elevatorSim.Update(20_ms);
    m_simTimer.Reset();
}
void Elevator::UseOutput(double output, State setpoint)
{
    // Calculate the feedforward from the sepoint
}
units::meter_t Elevator::GetEncoderDistance(rev::spark::SparkRelativeEncoder encoder)
{
    return encoder.GetPosition() * 2.0 * std::numbers::pi * ElevatorConstants::kElevatorDrumRadius / ElevatorConstants::kElevatorGearing;
}
void Elevator::Disable()
{
    m_ElevatorState = ElevatorConstants::ElevatorState::DISABLED;
    m_motorLeft.StopMotor();
    m_motorRight.StopMotor();
}
bool Elevator::IsHolding()
{
    return m_ElevatorState == ElevatorConstants::HOLDING;
}
void Elevator::Zero()
{
    m_ElevatorState = ElevatorConstants::ZEROING;
}

/*  Private Methods */

double Elevator::GetFusedHeight()
{
    // average the left and right encoder distance to get the encoder estimated height
    // note that the value represents double the actual height of stage 1
    double encoderHeight = GetEncoderHeight();

    double stage1Height = encoderHeight / 2.0;
    // apply the previously determined height correction to get an estimate of the
    // height of stage2 (carriage) relative to stage 1
    double stage2HeightEstimate = stage1Height - m_heightCorrection;

    // TODO detect a fault state if we definitely should be within range
    // of a magnet, but we aren't OR if we definitely should NOT be within
    // range and we are.  Condition should persist for X milliseconds.

    // using the estimated height, get the height of stage2 (carriage) relative to
    // stage 1
    double stage2Height = GetHallHeight(stage2HeightEstimate);
    // if no data, return the estimated total height
    if (stage2Height < -999.0)
        return stage1Height + stage2HeightEstimate;
    // otherwise return the true total height
    return stage1Height + stage2Height;
}

double Elevator::GetEncoderHeight()
{
    // average the left and right encoder distance to get the encoder estimated height
    return ((GetEncoderDistance(m_encoderLeft) + GetEncoderDistance(m_encoderRight)) / 2.0).value();
}

// returns the vertical velocity of the carriage (relative to ground)
double Elevator::GetEncoderVelocity()
{
    // average the two encoder velocities (returns rpm)
    double rpm = (m_encoderLeft.GetVelocity() + m_encoderRight.GetVelocity()) / 2.0;
    return rpm / 60.0 * 2.0 * std::numbers::pi * ElevatorConstants::kElevatorDrumRadius.value() / ElevatorConstants::kElevatorGearing;
}

// returns the height (in meters) of the carriage relate to stage 1, as determined
// by the hall sensor
// NOTE returns -1000.0 if no data is available, -2000.0 if fault is detected
double Elevator::GetHallHeight(double heightEstimate)
{
    // find the closest magnet holder using the height estimate
    double dist;
    int index = -1;
    double mindist = 1000.0;
    for (int i = 0; i < ElevatorConstants::kHallMagnetHolderCount; i++)
    {
        dist = std::abs(heightEstimate - ElevatorConstants::kHallMagnetHeights[i]);
        if (dist < mindist)
        {
            mindist = dist;
            index = i;
        }
    }
    double positionEstimate = heightEstimate - ElevatorConstants::kHallMagnetHeights[index];
    int magnetCount = ElevatorConstants::kHallMagnetCounts[index];
    double position = GetHallPosition(positionEstimate, magnetCount);
    frc::SmartDashboard::PutNumber("/Elevator/ELEVATOR_HALL_MAG_INDEX", index);
    frc::SmartDashboard::PutNumber("/Elevator/ELEVATOR_HALL_MAG_POSITION", position);
    if (position < -999.0)
        return position;
    else
        return ElevatorConstants::kHallMagnetHeights[index] + position;
}

// gets the raw position (duty cycle) from the candi
double Elevator::GetHallPWM()
{
    ctre::phoenix6::StatusSignal<units::angle::turn_t> signal = m_candi.GetPWM1Position(true);
    return signal.GetValueAsDouble();
}

// gets the raw proximity value (duty cycle) from the candi
double Elevator::GetProximityPWM()
{
    ctre::phoenix6::StatusSignal<units::angle::turn_t> signal = m_candi.GetPWM2Position(true);
    return signal.GetValueAsDouble();
}

// returns the position of the magnet holder center relative to the hall sensor center in meters
// position_estimate is used to determine the integer part of the mag field rotation
// position_estimate must be accurate to within +/-0.75 inches (0.5 rotation of the mag field)
// NOTE returns -1000.0 if no data is available, -2000.0 if sensor fault is detected
double Elevator::GetHallPosition(double positionEstimate, int magnetCount)
{
    // get the raw hall pwm value
    double pwm = m_hallPwm;
    // normalize to positive value from 0 to 1.0 in case candi is counting revolutions
    pwm = (pwm < 0 ? -1 : 1) * pwm;
    pwm -= std::trunc(pwm);
    // pwm < 0.025 or pwm > 0.95 represents fault
    // 0.025 < pwm < 0.09 represents no data yet
    // 0.91 < pwm < 0.95 indeterminate, ignore
    if (pwm < 0.025 or pwm > 0.95)
        return -2000.0;
    if (pwm < 0.09 || pwm > 0.91)
        return -1000.0;
    // convert the pwm duty cycle to an angle in rotations
    double rot = (pwm - 0.1) / 0.8;
    // clamp rot to 0 to 1
    if (rot < 0.0)
        rot = 0.0;
    if (rot > 1.0)
        rot = 1.0;

    // estimate the angle from the position estimate using the inverse of the piecewise linear function
    double rotEstimate;
    switch (magnetCount)
    {
    case 4:
        // if the estimated position is outside the interpolation range, return no data
        if (positionEstimate < ElevatorConstants::kHall4PwlPosition[0] ||
            positionEstimate > ElevatorConstants::kHall4PwlPosition[ElevatorConstants::kHallPwlPoints - 1])
            return -1000.0;
        rotEstimate = InterpolatePWL(ElevatorConstants::kHall4PwlPosition,
                                     ElevatorConstants::kHall4PwlAngle,
                                     ElevatorConstants::kHallPwlPoints, positionEstimate);
        break;
    default:
        // if the estimated position is outside the interpolation range, return no data
        if (positionEstimate < ElevatorConstants::kHall2PwlPosition[0] ||
            positionEstimate > ElevatorConstants::kHall2PwlPosition[ElevatorConstants::kHallPwlPoints - 1])
            return -1000.0;
        rotEstimate = InterpolatePWL(ElevatorConstants::kHall2PwlPosition,
                                     ElevatorConstants::kHall2PwlAngle,
                                     ElevatorConstants::kHallPwlPoints, positionEstimate);
        break;
    }
    // use the estimated angle to adjust the measured angle beyond a single rotation
    rot += std::trunc(rotEstimate);
    if (rot - rotEstimate > 0.5)
        rot -= 1.0;
    else
    {
        if (rot - rotEstimate < -0.5)
            rot += 1.0;
    }

    // interpolate the piecewise linear function at the angle to find the final position
    switch (magnetCount)
    {
    case 4:
        // if the estimated position is outside the interpolation range, return no data
        if (rot < ElevatorConstants::kHall4PwlAngle[0] ||
            rot > ElevatorConstants::kHall4PwlAngle[ElevatorConstants::kHallPwlPoints - 1])
            return -1000.0;
        return InterpolatePWL(ElevatorConstants::kHall4PwlAngle,
                              ElevatorConstants::kHall4PwlPosition,
                              ElevatorConstants::kHallPwlPoints, rot);
        break;
    default:
        // if the estimated position is outside the interpolation range, return no data
        if (rot < ElevatorConstants::kHall2PwlAngle[0] ||
            rot > ElevatorConstants::kHall2PwlAngle[ElevatorConstants::kHallPwlPoints - 1])
            return -1000.0;
        return InterpolatePWL(ElevatorConstants::kHall2PwlAngle,
                              ElevatorConstants::kHall2PwlPosition,
                              ElevatorConstants::kHallPwlPoints, rot);
        break;
    }
}

// interpolate the piecewise linear function at x given the break points xs, ys
// result will be extrapolated if outside range of xs
double Elevator::InterpolatePWL(const double *xs, const double *ys, int count, double x)
{
    double dx;
    double dy;

    for (int i = 0; i < count - 1; i++)
    {
        if (x <= xs[i + 1] || i == count - 2)
        {
            dx = xs[i + 1] - xs[i];
            dy = ys[i + 1] - ys[i];
            return ys[i] + dy * (x - xs[i]) / dx;
        }
    }
}

void Elevator::AutoCalibrateHeight()
{
    if (units::meters_per_second_t{std::abs(GetEncoderVelocity())} < ElevatorConstants::kAutoCalMaxVelocity)
    {
        frc::SmartDashboard::PutNumber("/Elevator/Calibrate vel", 10);
        double encoderHeight = GetEncoderHeight();
        double stage1Height = encoderHeight / 2.0;
        if (units::meter_t{stage1Height} > ElevatorConstants::kAutoCalMinHeight)
        {
            frc::SmartDashboard::PutNumber("/Elevator/Calibrate stage", 10);
            // apply the previously determined height correction to get an estimate of the
            // height of stage2 (carriage) relative to stage 1
            double stage2HeightEstimate = stage1Height - m_heightCorrection;

            // using the estimated height, get the height of stage2 (carriage) relative to
            // stage 1
            double stage2Height = GetHallHeight(stage2HeightEstimate);
            frc::SmartDashboard::PutNumber("/Elevator/Calibration height", stage2Height);
            // if no data, return the estimated total height
            if (stage2Height > -999.0)
            {
                frc::SmartDashboard::PutNumber("/Elevator/Calibrate valid", 10);
                m_heightCorrection = stage1Height - stage2Height;
                if (m_heightCorrection > ElevatorConstants::kMaxHallCalibration || m_heightCorrection < -ElevatorConstants::kMaxHallCalibration)
                {
                    m_heightCorrection = ElevatorConstants::kInitialHeightCorrection.value();
                    frc::SmartDashboard::PutBoolean("/Elevator/Elevator Correction Failure", true);
                }
            }
            frc::SmartDashboard::PutNumber("/Elevator/Elevator Height Correction", m_heightCorrection);
        }
    }
}