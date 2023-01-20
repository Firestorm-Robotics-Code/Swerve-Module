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
    SwerveModule frontLeftSwerve (FRONT_LEFT_SPEED,  FRONT_LEFT_DIREC, FRONT_LEFT_CANCODER,   1);         // Speed, direction, cancoder, and state
    SwerveModule frontRightSwerve (FRONT_RIGHT_SPEED, FRONT_RIGHT_DIREC, FRONT_RIGHT_CANCODER, 2);
    SwerveModule backRightSwerve (BACK_RIGHT_SPEED,  BACK_RIGHT_DIREC,  BACK_RIGHT_CANCODER,  3);
    SwerveModule mainSwerve (BACK_LEFT_SPEED,   BACK_LEFT_DIREC,  BACK_LEFT_CANCODER,    4);

public: 
    void Init() {
      setData("Tests", "FRC", 6341);
    }

    void BeginTeleop() {
      mainSwerve -> notZeroed();               // Tells the swerve to zero when called to
    }

    void TeleopLoop() {
      frc::SmartDashboard::PutNumber("FrontLeft CANCoder position", frontLeftSwerve -> GetAbsoluteCANCoderPos());
      frc::SmartDashboard::PutNumber("Frontright CANCoder position", frontRightSwerve -> GetAbsoluteCANCoderPos());
      frc::SmartDashboard::PutNumber("Backright CANCoder position", backRightSwerve -> GetAbsoluteCANCoderPos());
      frc::SmartDashboard::PutNumber("Backleft CANCoderPosition", mainSwerve -> GetAbsoluteCANCoderPos());
      frontLeftSwerve.Set(0.1);
      backLeftSwerve.Set(0.1);
      frontRightSwerve.Set(0.1);
      mainSwerve.Set(0.1);
    }
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
