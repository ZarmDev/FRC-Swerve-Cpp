// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#include <frc/AnalogGyro.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StructTopic.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/StringTopic.h>
#include <wpi/array.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <units/velocity.h>

#include "SwerveModule.h"

using namespace frc;
using namespace nt;

/**
 * Represents a swerve drive style drivetrain.
 */
class Drivetrain : public frc2::SubsystemBase
{
public:
    Drivetrain()
    {
        m_gyro.Reset();
        m_gyro.InitGyro();
        testPublisher = nt::NetworkTableInstance::GetDefault().GetStringTopic("test").Publish();
        estimatedPosePublisher = nt::NetworkTableInstance::GetDefault().GetStructTopic<Pose2d>("EstimatedPose").Publish();
        robotPosePublisher = nt::NetworkTableInstance::GetDefault().GetStructTopic<Pose2d>("RobotPose").Publish();
    }
    void Drive(units::meters_per_second_t xSpeed,
               units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
               bool fieldRelative, units::second_t period);
    void UpdateOdometry(const wpi::array<frc::SwerveModulePosition, 4U> &wheelPositions);
    void UpdatePoseEstimation();
    void UpdateSimulation(units::second_t period);
    // Pose2d GetPose();

    static constexpr auto kMaxSpeed = 3.0_mps; // 3 meters per second
    static constexpr units::radians_per_second_t kMaxAngularSpeed{
        std::numbers::pi}; // 1/2 rotation per second

private:
    StructPublisher<Pose2d> estimatedPosePublisher;
    StructPublisher<Pose2d> robotPosePublisher;
    nt::StringPublisher testPublisher;
    Translation2d m_frontLeftLocation{+0.381_m, +0.381_m};
    Translation2d m_frontRightLocation{+0.381_m, -0.381_m};
    Translation2d m_backLeftLocation{-0.381_m, +0.381_m};
    Translation2d m_backRightLocation{-0.381_m, -0.381_m};

    SwerveModule m_frontLeft{1, 2, 0, 1, 2, 3};
    SwerveModule m_frontRight{3, 4, 4, 5, 6, 7};
    SwerveModule m_backLeft{5, 6, 8, 9, 10, 11};
    SwerveModule m_backRight{7, 8, 12, 13, 14, 15};

    AnalogGyro m_gyro{0};

    SwerveDriveKinematics<4> m_kinematics{
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
        m_backRightLocation};

    // Gains are for example purposes only - must be determined for your own
    // robot!
    SwerveDrivePoseEstimator<4> m_poseEstimator{
        m_kinematics,
        Rotation2d{},
        {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
         m_backLeft.GetPosition(), m_backRight.GetPosition()},
        Pose2d{},
        {0.1, 0.1, 0.1},
        {0.1, 0.1, 0.1}};

    // https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-odometry.html
    frc::SwerveDriveOdometry<4> odometry{m_kinematics, m_gyro.GetRotation2d(), {m_frontLeft.GetPosition(), m_frontRight.GetPosition(), m_backLeft.GetPosition(), m_backRight.GetPosition()}, frc::Pose2d{5_m, 13.5_m, 0_rad}};
};
