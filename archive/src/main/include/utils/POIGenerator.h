// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Field2d.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Quaternion.h>
#include <frc/geometry/Rotation3d.h>
#include <networktables/StructTopic.h>
#include <frc/geometry/struct/Pose2dStruct.h>
#include <networktables/StructArrayTopic.h>

class POIGenerator
{
public:
  POIGenerator();
  void MakePOI();
  static frc::Pose2d GetPOI(std::string poiKey);
  void RemovePOI();
  frc::Pose2d GetClosestPOI();
  static frc::Pose2d DoubleArrayToPose2d(std::vector<double> arr)
  {
    if (arr.size() > 0)
    {
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

      frc::SmartDashboard::PutBoolean("DIDIWORK?", true);

      return frc::Pose2d(x, y, o);
    }
    frc::SmartDashboard::PutBoolean("DIDIWORK?", false);
    return frc::Pose2d(0_m, 0_m, 0_rad);
  };

private:
  std::string_view robotPoseLink = "base_link";
  nt::StructPublisher<frc::Pose2d> poiPublisher;
  nt::DoubleArrayPublisher poiPublisherDeprecated;
  std::vector<frc::Pose2d> poses{};
  frc::Pose2d closestPOI; // Field object
  nt::NetworkTableInstance networkTableInst;
  nt::DoubleArraySubscriber baseLinkSubscriber;
};
