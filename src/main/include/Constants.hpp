// Copyright (c) FRC Team 122. All Rights Reserved.

#pragma once
#include <cmath>
#include <cstdlib>
#include <numbers>

#include <ctre/phoenix6/controls/NeutralOut.hpp>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/math.h>
#include <units/time.h>

#include <frc/Preferences.h>
#include <units/velocity.h>

#include "SDSModuleType.hpp"

namespace ElectricalConstants
{
    const int kFrontLeftDriveMotorID = 10;
    const int kFrontLeftTurnMotorID = 11;
    const int kFrontLeftEncoderID = 12;

    const int kFrontRightDriveMotorID = 20;
    const int kFrontRightTurnMotorID = 21;
    const int kFrontRightEncoderID = 22;

    const int kBackLeftDriveMotorID = 30;
    const int kBackLeftTurnMotorID = 31;
    const int kBackLeftEncoderID = 32;

    const int kBackRightDriveMotorID = 40;
    const int kBackRightTurnMotorID = 41;
    const int kBackRightEncoderID = 42;

} // namespace ElectricalConstants

namespace DriveConstants
{
    const int kDriverPort = 0;
    const int kOperatorPort = 1;
    const int kDebugControllerPort = 2;

    const SDSModuleType mk4i_l1{0.10033,
                                (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0), true,
                                (14.0 / 50.0) * (10.0 / 60.0), false};
    const SDSModuleType mk4i_l2{0.10033,
                                (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0), true,
                                (14.0 / 50.0) * (10.0 / 60.0), false};
    const SDSModuleType mk4i_l3{0.10033,
                                (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0), true,
                                (14.0 / 50.0) * (10.0 / 60.0), false};
    const SDSModuleType mk4i_l1plus{0.10033,
                                    (16.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0), true,
                                    (14.0 / 50.0) * (10.0 / 60.0), false};
    const SDSModuleType mk4i_l2plus{0.10033,
                                    (16.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0), true,
                                    (14.0 / 50.0) * (10.0 / 60.0), false};
    const SDSModuleType mk4i_l3plus{0.10033,
                                    (16.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0), true,
                                    (14.0 / 50.0) * (10.0 / 60.0), false};
    // const std::map<std::string, SDSModuleType> moduleMap{{"mk4i_l1", mk4i_l1}, {"mk4i_l2", mk4i_l2}, {"mk4i_l3", mk4i_l3}};
    // inline constexpr std::string_view mk4i_l1 = "mk4i_l1";
    // inline constexpr std::string_view mk4i_l2 = "mk4i_l2";
    // inline constexpr std::string_view mk4i_l3 = "mk4i_l3";

    // const SDSModuleType kSDSModule = mk4i_l3;
    const SDSModuleType kSDSModule = mk4i_l1plus;

    const auto kTrackwidthMeters = 0.4_m; // Add to shuffleboard
    const auto kWheelbaseMeters = 0.4_m;  // Add to shuffleboard

    const double kDefaultAxisDeadband = 0.15;
    const units::meters_per_second_t kMaxTranslationalVelocity{4}; // Add to shuffleboard

    const units::radians_per_second_t kMaxRotationalVelocity{4}; // Add to shuffleboard
    const bool kIsFieldRelative = true;

    // ------------------------- MODULE OFFSETS -------------------------
    const frc::Rotation2d kFrontLeftOffset{-units::degree_t{0}}; // Add to shuffleboard
    inline constexpr std::string_view kFrontLeftOffsetKey = "kFrontLeftOffset";
    // ^ Module 1 ^ //
    const frc::Rotation2d kFrontRightOffset{-units::degree_t{0}}; // Add to shuffleboard
    inline constexpr std::string_view kFrontRightOffsetKey = "kFrontRightOffset";
    // ^ Module 2 ^ //
    const frc::Rotation2d kBackLeftOffset{-units::degree_t{0}}; // Add to shuffleboard
    inline constexpr std::string_view kBackLeftOffsetKey = "kBackLeftOffset";
    // ^ Module 3 ^ //
    const frc::Rotation2d kBackRightOffset{-units::degree_t{0}}; // Add to shuffleboard
    inline constexpr std::string_view kBackRightOffsetKey = "kBackRightOffset";
    // ^ Module 4 ^ //

    const frc::Translation2d kFrontLeftPosition =
        frc::Translation2d(units::meter_t{kTrackwidthMeters / 2.0},
                           units::meter_t{kWheelbaseMeters / 2.0});
    const frc::Translation2d kFrontRightPosition =
        frc::Translation2d(units::meter_t{kTrackwidthMeters / 2.0},
                           units::meter_t{-kWheelbaseMeters / 2.0});
    const frc::Translation2d kBackLeftPosition =
        frc::Translation2d(units::meter_t{-kTrackwidthMeters / 2.0},
                           units::meter_t{kWheelbaseMeters / 2.0});
    const frc::Translation2d kBackRightPosition =
        frc::Translation2d(units::meter_t{-kTrackwidthMeters / 2.0},
                           units::meter_t{-kWheelbaseMeters / 2.0});
} // namespace DriveConstants

namespace ModuleConstants
{
    // meters / second
    const auto kMaxSpeed = DriveConstants::kMaxTranslationalVelocity;
    // meters
    const auto kWheelDiameterMeters =
        units::meter_t{0.092815210491};
    // meters / turn

    const auto kWheelEffectiveDiameterCoeff = 1.11016;
    // const auto kWheelEffectiveDiameterCoeff = 1.08426;
    // const auto kWheelEffectiveDiameterCoeff = 0.97561;

    const auto kWheelCircumference =
        kWheelDiameterMeters * std::numbers::pi * kWheelEffectiveDiameterCoeff / units::turn_t{1.0};

    // ratio is motor rot / wheel rot
    const double kDriveGearRatio = 1.0 / DriveConstants::kSDSModule.driveReduction;
    const double kTurnGearRatio = 1.0 / DriveConstants::kSDSModule.steerReduction;

    const auto kDriveConversion = kWheelCircumference / kDriveGearRatio;

    const bool kDriveMotorInverted = DriveConstants::kSDSModule.driveInverted;

    const bool kSteerMotorInverted = true;

    const auto kDriveMotorNeutral =
        ctre::phoenix6::signals::NeutralModeValue::Brake;
    const auto kSteerMotorNeutral =
        ctre::phoenix6::signals::NeutralModeValue::Brake; // set back to brake to be amazing
    const bool kEncoderInverted = false;

    const bool kSteerEnableCurrentLimit = true;
    const int kSteerContinuousCurrentLimit = 25;
    const int kSteerPeakCurrentLimit = 30;
    const double kSteerPeakCurrentDuration = 0.1;

    const bool kDriveEnableCurrentLimit = true;
    const int kDriveContinuousCurrentLimit = 35;
    const int kDrivePeakCurrentLimit = 80;
    const double kDrivePeakCurrentDuration = 0.1;

    // TODO: retune constants
    const double kDriveP = 0.2;
    const double kDriveI = 0.0;
    const double kDriveD = 0.0;
    const double kDriveS = 0.02496863326; // Volts
    const double kDriveV = 0.1089791826;  // Volts / (rot / s)
    const double kDriveA = 0.0;           // Volts / (rot / s^2)

    // const double kDriveS = 0.05558; // Volts
    // const double kDriveV = 0.20333; // Volts / (rot / s)
    // const double kDriveA = 0.02250; // Volts / (rot / s^2)

    const double kSteerP = 120.0; // TODO: ensure this works with new inversion
    const double kSteerI = 0.0;
    const double kSteerD = 0.37;

} // namespace ModuleConstants

namespace MathUtilNK
{
    inline double calculateAxis(double axis, double deadband)
    {
        if (std::abs(axis) > deadband)
        {
            return (axis - std::copysign(deadband, axis)) / (1.0 - deadband);
        }
        else
        {
            return 0.0;
        }
    }

} // namespace MathUtilNK
