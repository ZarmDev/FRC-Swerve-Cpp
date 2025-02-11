// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/CommandXboxController.h>
#include <units/velocity.h>
#include <units/time.h>
#include <iostream>

#include "Drivetrain.h"

class Robot : public frc::TimedRobot
{
public:
  void AutonomousPeriodic() override
  {
    DriveWithJoystick(false);
    m_swerve.UpdatePoseEstimation();
  }

  void TeleopPeriodic() override
  {
    DriveWithJoystick(true);
  }

private:
  frc2::CommandXboxController m_controller{0};
  frc::GenericHID generic{0};
  Drivetrain m_swerve;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{3 / 1_s};

  void DriveWithJoystick(bool fieldRelative)
  {
    double getLeftY;
    double getLeftX;
    double getRightX;
    if (frc::RobotBase::IsSimulation()) {
      getLeftY = generic.GetRawAxis(0);
      getLeftX = generic.GetRawAxis(1);
      getRightX = generic.GetRawAxis(2);
      // std::cout << getLeftY << "\n" << getLeftX << "\n" << getRightX << '\n';
    } else {
      getLeftY = m_controller.GetLeftY();
      getLeftX = m_controller.GetLeftX();
      getRightX = m_controller.GetRightX();
    }
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    const auto xSpeed = -m_xspeedLimiter.Calculate(getLeftY) *
                        Drivetrain::kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    const auto ySpeed = -m_yspeedLimiter.Calculate(getLeftX) *
                        Drivetrain::kMaxSpeed;
    // std::cout << units::to_string(xSpeed) << " " << units::to_string(ySpeed) << '\n';
    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    const auto rot = -m_rotLimiter.Calculate(getRightX) *
                     Drivetrain::kMaxAngularSpeed;

    m_swerve.Drive(xSpeed, ySpeed, rot, fieldRelative, GetPeriod());
  }
};

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
