// Copyright (c) FRC Team 122. All Rights Reserved.

#include "subsystems/SwerveDrive.hpp"

SwerveDrive::SwerveDrive()
    : modules{{SwerveModule(ElectricalConstants::kFrontLeftDriveMotorID,
                            ElectricalConstants::kFrontLeftTurnMotorID,
                            ElectricalConstants::kFrontLeftEncoderID,
                            DriveConstants::kFrontLeftOffset),
               SwerveModule(ElectricalConstants::kFrontRightDriveMotorID,
                            ElectricalConstants::kFrontRightTurnMotorID,
                            ElectricalConstants::kFrontRightEncoderID,
                            DriveConstants::kFrontRightOffset),
               SwerveModule(ElectricalConstants::kBackLeftDriveMotorID,
                            ElectricalConstants::kBackLeftTurnMotorID,
                            ElectricalConstants::kBackLeftEncoderID,
                            DriveConstants::kBackLeftOffset),
               SwerveModule(ElectricalConstants::kBackRightDriveMotorID,
                            ElectricalConstants::kBackRightTurnMotorID,
                            ElectricalConstants::kBackRightEncoderID,
                            DriveConstants::kBackRightOffset)}},
      kSwerveKinematics{{DriveConstants::kFrontLeftPosition, DriveConstants::kFrontRightPosition,
                         DriveConstants::kBackLeftPosition, DriveConstants::kBackRightPosition}},
      pidX{0.9, 1e-4, 0}, pidY{0.9, 1e-4, 0}, pidRot{0.15, 0, 0}, networkTableInst(nt::NetworkTableInstance::GetDefault()), m_poseEstimator{kSwerveKinematics,
                                                                                                                                            frc::Rotation2d(m_pigeon.GetYaw().GetValue()), // TODO: YAW is CCW+ whereas this API is CW+ (Check if need to reverse)
                                                                                                                                            // frc::Rotation2d(units::radian_t{navx.GetYaw()}), // TODO: YAW is CCW+ whereas this API is CW+ (Check if need to reverse)
                                                                                                                                            {modules[0].GetPosition(), modules[1].GetPosition(), modules[2].GetPosition(),
                                                                                                                                             modules[3].GetPosition()},
                                                                                                                                            frc::Pose2d()},
      m_pigeonSim{m_pigeon}
{

    // Add a function that loads the Robot Preferences, including
    // offsets, Module positions, max speed, wheel diameter

    kSwerveKinematics = frc::SwerveDriveKinematics<4U>{
        {DriveConstants::kFrontLeftPosition, DriveConstants::kFrontRightPosition,
         DriveConstants::kBackLeftPosition, DriveConstants::kBackRightPosition}};
    // Function should
    // frc::Preferences::InitDouble(kArmPositionKey, number);
    // double x =  frc::Preferences::GetDouble(kArmPositionKey, m_armSetpoint.value())}
    // degrees_t x_deg = degrees_t{x} (Psuedo code)

    // navx.Calibrate();
    navx.Reset();

    speeds = frc::ChassisSpeeds();
    networkTableInst.StartServer();
    frc::SmartDashboard::PutData("Field", &m_field);
    auto visionStdDevs = wpi::array<double, 3U>{0.2, 0.2, 0.9};
    m_poseEstimator.SetVisionMeasurementStdDevs(visionStdDevs);

    m_visionPoseEstimator = PoseEstimator();

    timer.Start();

    poseTable = networkTableInst.GetTable("ROS2Bridge");
    baseLink1Subscribe = poseTable->GetDoubleArrayTopic(baseLink1).Subscribe(
        {}, {.periodic = 0.01, .sendAll = true});
    baseLink2Subscribe = poseTable->GetDoubleArrayTopic(baseLink2).Subscribe(
        {}, {.periodic = 0.01, .sendAll = true});
    visionStdDevSub = poseTable->GetDoubleArrayTopic(visionStdDev).Subscribe({}, {.periodic = 0.01, .sendAll = true});

    baseLinkPublisher = poseTable->GetDoubleArrayTopic(baseLink).Publish();
    timePublisher = poseTable->GetDoubleArrayTopic(timeLinkName).Publish();

    SetOffsets();

    pathplanner::RobotConfig pathplannerConfig = pathplanner::RobotConfig::fromGUISettings();
    // Configure Auto Swerve
    pathplanner::AutoBuilder::configure(
        [this]()
        { return this->GetPose(); }, // Robot pose supplier

        [this](frc::Pose2d poseReset)
        {
            this->ResetPose(poseReset);
        }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this]()
        {
            frc::SmartDashboard::PutNumber("ac drive/vx", this->GetPose().X().value());
            return this->getRobotRelativeSpeeds();
        }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](frc::ChassisSpeeds speedsRelative)
        {
            this->Drive(speedsRelative);
        },                                                         // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        std::make_shared<pathplanner::PPHolonomicDriveController>( // PPHolonomicController is the built in path following
                                                                   // controller for holonomic drive trains
            pathplanner::PIDConstants(5, 0.0, 0.0),                // Translation PID constants
            pathplanner::PIDConstants(5, 0.0, 0.0)                 // Rotation PID constants
            ),
        pathplannerConfig,
        []()
        {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            auto alliance = frc::DriverStation::GetAlliance();
            if (alliance)
            {
                return alliance.value() == frc::DriverStation::Alliance::kRed;
            }
            return false;
        },
        this // Reference to this subsystem to set requirements
    );

    if constexpr (frc::RobotBase::IsSimulation())
    {
        m_simTimer.Start();
    }
}

// void SwerveDrive::InitPreferences()
// {
//     frc::Preferences::InitDouble(DriveConstants::kBackLeftOffsetKey,
//                                  DriveConstants::kBackLeftOffset.Radians().value());
// }

// void SwerveDrive::GetPrefernces()
// {
//     auto kBackLeftOffsetDouble = frc::Preferences::GetDouble(DriveConstants::kBackLeftOffsetKey,
//                                                              DriveConstants::kBackLeftOffset.Radians().value());

//     modules = std::array<SwerveModule, 4>{
//         {SwerveModule(ElectricalConstants::kFrontLeftDriveMotorID, ElectricalConstants::kFrontLeftTurnMotorID,
//                       ElectricalConstants::kFrontLeftEncoderID, DriveConstants::kFrontLeftOffset),
//          SwerveModule(ElectricalConstants::kFrontRightDriveMotorID, ElectricalConstants::kFrontRightTurnMotorID,
//                       ElectricalConstants::kFrontRightEncoderID, DriveConstants::kFrontRightOffset),
//          SwerveModule(ElectricalConstants::kBackLeftDriveMotorID, ElectricalConstants::kBackLeftTurnMotorID,
//                       ElectricalConstants::kBackLeftEncoderID, frc::Rotation2d(units::radian_t{kBackLeftOffsetDouble})),
//          SwerveModule(ElectricalConstants::kBackRightDriveMotorID, ElectricalConstants::kBackRightTurnMotorID,
//                       ElectricalConstants::kBackRightEncoderID, DriveConstants::kBackRightOffset)}};

//     m_poseEstimator = frc::SwerveDrivePoseEstimator<4U>{
//         kSwerveKinematics,
//         frc::Rotation2d(units::degree_t{m_pigeon.GetAngle()}),
//         {modules[0].GetPosition(), modules[1].GetPosition(), modules[2].GetPosition(), modules[3].GetPosition()},
//         frc::Pose2d()};
// }

// This method will be called once per scheduler run
void SwerveDrive::Periodic()
{
    // getCameraResults();
    // sensor fusion? EKF (eek kinda fun) (extended Kalman filter)

    PublishOdometry(m_poseEstimator.GetEstimatedPosition());
    if (useVision)
    {
        UpdatePoseEstimate();
    }
    m_poseEstimator.Update(m_pigeon.GetRotation2d(), GetModulePositions());
    m_field.SetRobotPose(m_poseEstimator.GetEstimatedPosition());

    // PrintNetworkTableValues();

    frc::SmartDashboard::PutNumber("Heading", GetHeading().Degrees().value());
    // UpdateOdometry();
    PeriodicShuffleboard();
}

void SwerveDrive::SimulationPeriodic()
{

    units::second_t dt = m_simTimer.Get();
    m_simTimer.Reset();
    units::angle::degree_t delta = m_pigeon.GetAngularVelocityZWorld().GetValue() * dt;
    m_pigeonSim.AddYaw(delta);
}

void SwerveDrive::Drive(frc::ChassisSpeeds speeds)
{
    if (enable)
    {
        auto dT = 20_ms;
        timer.Reset();

        auto prevVX = frc::SmartDashboard::GetNumber("drive/vx", 0.0);
        auto prevVY = frc::SmartDashboard::GetNumber("drive/vy", 0.0);
        double accelLimit = frc::SmartDashboard::GetNumber("drive/accelLim", 2.0);

        double velocityCommanded = std::sqrt(std::pow(speeds.vx.value(), 2) + std::pow(speeds.vy.value(), 2));
        double prevVelocity = std::sqrt(std::pow(prevVX, 2) + std::pow(prevVY, 2));

        velocityCommanded = std::min(accelLimit * (0.02) + (prevVelocity), velocityCommanded);

        Eigen::Vector2d prevVelocityVector(speeds.vx.value(), speeds.vy.value());
        Eigen::Vector2d a = prevVelocityVector.normalized() * velocityCommanded;

        speeds.vx = units::velocity::meters_per_second_t(a[0]);
        speeds.vy = units::velocity::meters_per_second_t(a[1]);

        auto states = kSwerveKinematics.ToSwerveModuleStates(speeds);

        kSwerveKinematics.DesaturateWheelSpeeds(
            &states, speeds, units::meters_per_second_t{ModuleConstants::kMaxSpeed},
            DriveConstants::kMaxTranslationalVelocity, DriveConstants::kMaxRotationalVelocity);

        for (int i = 0; i < 4; i++)
        {
            modules[i].SetDesiredState(states[i]);
        }

        if constexpr (frc::RobotBase::IsSimulation())
        {
            m_pigeonSim.SetAngularVelocityZ(speeds.omega);
        }

        frc::SmartDashboard::PutNumber("drive/vx", speeds.vx.value());
        frc::SmartDashboard::PutNumber("drive/vy", speeds.vy.value());
        frc::SmartDashboard::PutNumber("drive/omega", speeds.omega.value());
    }
}

void SwerveDrive::Strafe(frc::ChassisSpeeds s_speeds, double desiredAngle)
{
    auto currentAngle = m_poseEstimator.GetEstimatedPosition().Rotation().Radians().value();

    double errorBand = (M_PI - (-M_PI)) / 2;
    pos_Error = frc::InputModulus(desiredAngle - currentAngle, -errorBand, errorBand);

    s_speeds.omega = units::angular_velocity::radians_per_second_t{9.5 * (pos_Error)};
    frc::SmartDashboard::PutNumber("wrapped cA", currentAngle);
    frc::SmartDashboard::PutNumber("wrapped dA", desiredAngle);

    auto states = kSwerveKinematics.ToSwerveModuleStates(s_speeds);

    kSwerveKinematics.DesaturateWheelSpeeds(
        &states, s_speeds, units::meters_per_second_t{ModuleConstants::kMaxSpeed},
        DriveConstants::kMaxTranslationalVelocity, units::radians_per_second_t{0.5});

    for (int i = 0; i < 4; i++)
    {
        modules[i].SetDesiredState(states[i]);
    }

    priorSpeeds = s_speeds;
}

void SwerveDrive::SetFast() {}

void SwerveDrive::SetSlow() {}

frc::Rotation2d SwerveDrive::GetHeading()
{
    return m_pigeon.GetRotation2d();
    // return navx.GetRotation2d();
}

void SwerveDrive::ResetHeading()
{
    if (enable == true)
    {
        m_pigeon.Reset();
        // navx.Reset();
    }
}

void SwerveDrive::ResetDriveEncoders()
{
    for (auto &module : modules)
    {
        module.ResetDriveEncoders();
    }
}

std::array<frc::SwerveModulePosition, 4> SwerveDrive::GetModulePositions()
{
    return std::array<frc::SwerveModulePosition, 4>{
        {modules[0].GetPosition(), modules[1].GetPosition(), modules[2].GetPosition(),
         modules[3].GetPosition()}};
}

void SwerveDrive::ResetPose(frc::Pose2d position)
{
    // odometry.ResetPosition(GetHeading(), GetModulePositions(), position);
    m_poseEstimator.ResetPosition(GetHeading(), GetModulePositions(), position);
}

frc::Pose2d SwerveDrive::GetPose()
{
    return m_poseEstimator.GetEstimatedPosition();
}

void SwerveDrive::UpdateOdometry()
{
    // odometry.Update(GetHeading(), GetModulePositions());
}
frc::ChassisSpeeds SwerveDrive::getRobotRelativeSpeeds()
{
    auto temp = kSwerveKinematics.ToChassisSpeeds(
        {modules[0].GetCurrentState(), modules[1].GetCurrentState(), modules[2].GetCurrentState(),
         modules[3].GetCurrentState()});

    return temp;
}

void SwerveDrive::InitializePID()
{
    pidX = frc::PIDController(0.9, 1e-4, 0);
    pidY = frc::PIDController(0.9, 1e-4, 0);
    pidRot = frc::PIDController(0.15, 0, 0);

    pidX.SetTolerance(0.025);
    pidY.SetTolerance(0.025);
    pidRot.SetTolerance(1);

    hasRun = false;
}

void SwerveDrive::SetReference(frc::Pose2d desiredPose)
{
    if ((!pidX.AtSetpoint() && !pidY.AtSetpoint()) | !hasRun)
    {
        speeds =
            frc::ChassisSpeeds{units::meters_per_second_t{
                                   pidX.Calculate(GetPose().X().value(), desiredPose.X().value())},
                               units::meters_per_second_t{
                                   pidY.Calculate(GetPose().Y().value(), desiredPose.Y().value())},
                               units::radians_per_second_t{0}};
        Drive(speeds);
    }
}

//--------------------------------------------

void SwerveDrive::UpdatePoseEstimate()
{
    auto result1 = baseLink1Subscribe.GetAtomic();
    auto result2 = baseLink2Subscribe.GetAtomic();
    auto resultStdDev = visionStdDevSub.GetAtomic();
    frc::SmartDashboard::PutBoolean("Vision", false);

    if (resultStdDev.value.size() > 0)
    {
        m_poseEstimator.SetVisionMeasurementStdDevs({resultStdDev.value[0], resultStdDev.value[1], resultStdDev.value[2]});
    }
    else
    {
        m_poseEstimator.SetVisionMeasurementStdDevs({1.0, 1.0, 1.0});
    }

    if (result1.value.size() > 0)
    {
        frc::SmartDashboard::PutBoolean("Vision", true);

        auto compressedResults = result1.value;
        rotation_q = frc::Quaternion(compressedResults.at(6), compressedResults.at(3),
                                     compressedResults.at(4), compressedResults.at(5));

        auto posTranslation = frc::Translation3d(units::meter_t{compressedResults.at(0)},
                                                 units::meter_t{compressedResults.at(1)},
                                                 units::meter_t{compressedResults.at(2)});
        frc::Pose3d cameraPose = frc::Pose3d(posTranslation, frc::Rotation3d(rotation_q));
        if (poseFilter1.IsPoseValid(cameraPose, compressedResults.at(7)))
        {
            frc::Pose2d visionMeasurement2d = cameraPose.ToPose2d();
            m_poseEstimator.AddVisionMeasurement(visionMeasurement2d,
                                                 units::second_t{compressedResults.at(7)});
        }
    }
    if (result2.value.size() > 0)
    {
        auto compressedResults = result2.value;
        rotation_q = frc::Quaternion(compressedResults.at(6), compressedResults.at(3),
                                     compressedResults.at(4), compressedResults.at(5));

        auto posTranslation = frc::Translation3d(units::meter_t{compressedResults.at(0)},
                                                 units::meter_t{compressedResults.at(1)},
                                                 units::meter_t{compressedResults.at(2)});
        frc::Pose3d cameraPose = frc::Pose3d(posTranslation, frc::Rotation3d(rotation_q));
        if (poseFilter2.IsPoseValid(cameraPose, compressedResults.at(7)))
        {
            frc::Pose2d visionMeasurement2d = cameraPose.ToPose2d();
            m_poseEstimator.AddVisionMeasurement(visionMeasurement2d,
                                                 units::second_t{compressedResults.at(7)});
        }
    }
    // m_poseEstimator.Update(navx.GetRotation2d(), GetModulePositions());
}

void SwerveDrive::PublishOdometry(frc::Pose2d odometryPose)
{
    double time = nt::Now() / (1e6);
    Eigen::Vector3d odoRotation =
        Eigen::Vector3d(0.0, 0.0, double(odometryPose.Rotation().Radians()));
    frc::Quaternion odoPoseQ = frc::Quaternion::FromRotationVector(odoRotation);
    double poseDeconstruct[]{double{odometryPose.X()},
                             double{odometryPose.Y()},
                             0.0,
                             odoPoseQ.X(),
                             odoPoseQ.Y(),
                             odoPoseQ.Z(),
                             odoPoseQ.W(),
                             time};
    baseLinkPublisher.Set(poseDeconstruct, time);
    double timearr[]{time};
    timePublisher.Set(timearr, time);
}

void SwerveDrive::EnableDrive()
{
    enable = true;
    // frc::SmartDashboard::PutBoolean("TestTestTest", enable);
}
void SwerveDrive::DisableDrive()
{
    enable = false;
    // frc::SmartDashboard::PutBoolean("TestTestTest", enable);
}

// Warning, this is unfinished (and also deprecated), and may produce results that are not expected
void SwerveDrive::WeightedDriving(bool approach, double leftXAxis,
                                  double leftYAxis, double rightXAxis, std::string poiKey)
{
    auto dT = timer.Get();
    timer.Reset();

    auto Po = frc::SmartDashboard::GetNumber("Note Po", 0.0);
    auto Px = frc::SmartDashboard::GetNumber("Note Px", 1);
    auto Py = frc::SmartDashboard::GetNumber("Note Py", 1);

    // TODO: Continue tuning
    frc::Pose2d Target;

    if (poiKey == "vision")
    {
        Target = m_visionPoseEstimator.GetMeasurement();
    }
    else
    {
        frc::Pose2d POI = poiGenerator.GetPOI(poiKey);
        Target = POI.RelativeTo(m_poseEstimator.GetEstimatedPosition());
    }

    auto targetX = Target.X();
    auto targetY = Target.Y();
    auto targetRotation = Target.Rotation().Radians().value();

    frc::SmartDashboard::PutNumber("TargetRotation", targetRotation);

    auto unsaturatedX = double(approach * targetX * Px);
    auto unsaturatedY = double(approach * targetY * Py);
    auto unsaturatedPO = double(approach * targetRotation * Po);
    // auto unsaturatedDO = double(approach * (TargetRotation - prevOError) / dT * Do);

    // prevOError = targetRotation;
    auto saturatedX = std::copysign(std::min(std::abs(unsaturatedX), 0.5), unsaturatedX);
    auto saturatedY = std::copysign(std::min(std::abs(unsaturatedY), 0.5), unsaturatedY);
    auto saturatedOmega = std::copysign(std::min(std::abs(unsaturatedPO), 0.4), unsaturatedPO);

    // Code with dampening
    // auto saturatedOmega = std::copysign(std::min(std::abs(unsaturatedPO - unsaturatedDO),
    //                                              frc::SmartDashboard::GetNumber("Note P", 0.4)),
    //                                     unsaturatedPO - unsaturatedDO);

    auto vx =
        units::meters_per_second_t(saturatedX +
                                   double(-leftXAxis * DriveConstants::kMaxTranslationalVelocity));

    auto vy =
        units::meters_per_second_t(saturatedY +
                                   double(-leftYAxis * DriveConstants::kMaxTranslationalVelocity));

    auto omega =
        units::radians_per_second_t(saturatedOmega +
                                    double(-rightXAxis * DriveConstants::kMaxRotationalVelocity));

    Drive(frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        vx,
        vy,
        omega,
        GetHeading()));
}

bool SwerveDrive::atSetpoint()
{
    if (pos_Error < 0.05)
    {
        return true;
    }
    return false;
}

void SwerveDrive::TurnVisionOff()
{
    useVision = false;
}

void SwerveDrive::TurnVisionOn()
{
    useVision = true;
}

void SwerveDrive::PeriodicShuffleboard()
{
    // auto VariableName = frc::SmartDashboard::GetString("SmartDashboard/Swerve/kFrontLeftOffset", "kFrontLeftOffset");
    // frc::SmartDashboard::PutString("SmartDashboard/Swerve/kFrontLeftOffset", VariableName);
    // frc::SmartDashboard::GetNumber("SmartDashboard/Swerve/WidthMeters", 0);
    // frc::SmartDashboard::GetNumber("SmartDashboard/Swerve/BaseMeters", 0);
    // frc::SmartDashboard::GetNumber("SmartDashboard/Swerve/MaxTranslationalVelocity", 0);
}

void SwerveDrive::ShuffleboardInit() {}

void SwerveDrive::SetOffsets()
{
    auto FrontLeftDegree = frc::SmartDashboard::GetNumber("FrontLeftDegree", -4.8);
    frc::SmartDashboard::SetPersistent("FrontLeftDegree");
    frc::Rotation2d kFrontLeftOffset(-units::degree_t{FrontLeftDegree});

    auto FrontRightDegree = frc::SmartDashboard::GetNumber("FrontRightDegree", -66);
    frc::SmartDashboard::SetPersistent("FrontRightDegree");
    frc::Rotation2d kFrontRightOffset(-units::degree_t{FrontRightDegree});

    auto BackLeftDegree = frc::SmartDashboard::GetNumber("BackLeftDegree", 70);
    frc::SmartDashboard::SetPersistent("BackLeftDegree");
    frc::Rotation2d kBackLeftOffset(-units::degree_t{BackLeftDegree});

    auto BackRightDegree = frc::SmartDashboard::GetNumber("BackRightDegree", 178);
    frc::SmartDashboard::SetPersistent("BackRightDegree");
    frc::Rotation2d kBackRightOffset(-units::degree_t{BackRightDegree});

    std::vector<frc::Rotation2d> offsets = {kFrontLeftOffset,
                                            kFrontRightOffset,
                                            kBackLeftOffset,
                                            kBackRightOffset};

    for (int i = 0; i < 4; i++)
    {
        modules[i].SetOffset(offsets[i]);
    }
}