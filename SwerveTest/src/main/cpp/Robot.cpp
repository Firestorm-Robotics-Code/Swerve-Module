// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/DriverStation.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <hal/DriverStation.h>
#include <networktables/NetworkTable.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "SwerveModule.hpp"
#include "ModularRobot.hpp"
#include "constants.h"


class Robot : public ModularRobot {
    SwerveModule* frontLeftSwerve  = new SwerveModule   {FRONT_LEFT_SPEED,  FRONT_LEFT_DIREC, FRONT_LEFT_CANCODER,  1};         // Speed, direction, cancoder, and state
    SwerveModule* frontRightSwerve = new SwerveModule   {FRONT_RIGHT_SPEED, FRONT_LEFT_DIREC, FRONT_RIGHT_CANCODER, 2};
    SwerveModule* backRightSwerve  = new SwerveModule   {BACK_RIGHT_SPEED,  BACK_LEFT_DIREC,  BACK_LEFT_CANCODER,   3};
    SwerveModule* mainSwerve       = new SwerveModule   {BACK_LEFT_SPEED,   BACK_LEFT_DIREC,  BACK_LEFT_CANCODER,   4};

public: 
    void Init() {

    }

    void BeginTeleop() {
      mainSwerve -> notZeroed();               // Tells the swerve to zero when called to
    }

    void TeleopLoop() {
      frc::SmartDashboard::PutNumber("FrontLeft CANCoder position", frontLeftSwerve -> GetAbsoluteCANCoderPos());
      frc::SmartDashboard::PutNumber("Frontright CANCoder position", frontRightSwerve -> GetAbsoluteCANCoderPos());
      frc::SmartDashboard::PutNumber("Backright CANCoder position", backRightSwerve -> GetAbsoluteCANCoderPos());
      frc::SmartDashboard::PutNumber("Backleft CANCoderPosition", mainSwerve -> GetAbsoluteCANCoderPos());
    }
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
