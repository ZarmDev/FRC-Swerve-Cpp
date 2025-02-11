// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drivetrain.h"

#include <frc/Timer.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <iostream>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include "ExampleGlobalMeasurementSensor.h"

using namespace units;
using namespace frc2;

void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rot, bool fieldRelative,
                       units::second_t period)
{
    wpi::array<frc::SwerveModuleState, 4U> states =
        m_kinematics.ToSwerveModuleStates(frc::ChassisSpeeds::Discretize(
            fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                                xSpeed, ySpeed, rot,
                                m_poseEstimator.GetEstimatedPosition().Rotation())
                          : frc::ChassisSpeeds{xSpeed, ySpeed, rot},
            period));

    m_kinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);

    auto [fl, fr, bl, br] = states;

    // for (const auto& state : states) {
    //     std::cout << "Speed: " << state.speed.value() << " m/s, "
    //               << "Angle: " << state.angle.Degrees().value() << " degrees" << std::endl;
    // }

    m_frontLeft.SetDesiredState(fl);
    m_frontRight.SetDesiredState(fr);
    m_backLeft.SetDesiredState(bl);
    m_backRight.SetDesiredState(br);

    // Update wheel positions
    wpi::array<frc::SwerveModulePosition, 4U> wheelPositions = {
        m_frontLeft.GetPosition(),
        m_frontRight.GetPosition(),
        m_backLeft.GetPosition(),
        m_backRight.GetPosition()
    };

    // Update odometry with the new wheel positions
    UpdateOdometry(wheelPositions);
}

void Drivetrain::UpdateOdometry(const wpi::array<frc::SwerveModulePosition, 4U> &wheelPositions)
{
    // const wpi::array<frc::SwerveModulePosition, 4U> wheelPositions = {m_frontLeft.GetPosition(), m_frontRight.GetPosition(), m_backLeft.GetPosition(), m_backRight.GetPosition()};
    frc::Rotation2d gyroAngle = m_gyro.GetRotation2d();
    odometry.Update(gyroAngle, wheelPositions);
    Pose2d robotPose = odometry.GetPose();
    // std::cout << units::to_string(m_frontLeft.GetPosition().distance) << '\n';
    robotPosePublisher.Set(robotPose);
}

void Drivetrain::UpdatePoseEstimation()
{
    m_poseEstimator.Update(m_gyro.GetRotation2d(),
                           {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                            m_backLeft.GetPosition(), m_backRight.GetPosition()});

    // Also apply vision measurements. We use 0.3 seconds in the past as an
    // example -- on a real robot, this must be calculated based either on latency
    // or timestamps.
    m_poseEstimator.AddVisionMeasurement(
        ExampleGlobalMeasurementSensor::GetEstimatedGlobalPose(
            m_poseEstimator.GetEstimatedPosition()),
        frc::Timer::GetTimestamp() - 0.3_s);
    frc::Pose2d estimatedPose = m_poseEstimator.GetEstimatedPosition();
    estimatedPosePublisher.Set(estimatedPose);
}
