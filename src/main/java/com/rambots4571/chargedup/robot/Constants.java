// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.rambots4571.chargedup.robot;

import com.rambots4571.chargedup.robot.utils.Alert;
import com.rambots4571.chargedup.robot.utils.COTSFalconSwerveConstants;
import com.rambots4571.chargedup.robot.utils.SwerveModuleConstants;
import com.rambots4571.chargedup.robot.utils.Alert.AlertType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.Map;

public final class Constants {

  public static class DriveConstants {
    ///////////////// CAN IDs /////////////////
    
    /* Front Left Module - Module 0 */
    public static final class Mod0 { //TODO: This must be tuned to specific robot
        public static final int driveMotorID = 1;
        public static final int angleMotorID = 2;
        public static final int canCoderID = 3;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 { //TODO: This must be tuned to specific robot
        public static final int driveMotorID = 4;
        public static final int angleMotorID = 5;
        public static final int canCoderID = 6;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
    
    /* Back Left Module - Module 2 */
    public static final class Mod2 { //TODO: This must be tuned to specific robot
        public static final int driveMotorID = 7;
        public static final int angleMotorID = 8;
        public static final int canCoderID = 9;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 { //TODO: This must be tuned to specific robot
      public static final int driveMotorID = 10;
      public static final int angleMotorID = 11;
      public static final int canCoderID = 12;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
      public static final SwerveModuleConstants constants = 
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    public static final int PIGEON_IMU_2 = 13;

    ///////////////// DRIVETRAIN CHARACTERISTICS /////////////////
    public static final COTSFalconSwerveConstants chosenModule = 
      COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

    public static final double kTrackWidthMeters = 0.42;
    public static final double kWheelBaseMeters = 0.73;
    public static final double kWheelCircumference = chosenModule.wheelCircumference;

    public static final double kDriveGearRatio = chosenModule.driveGearRatio;
    public static final double kTurnGrearRatio = chosenModule.angleGearRatio;

    public static final double kMaxSpeedMetersPerSecond = 3.0;

    public static final double kMaxVoltage = 12.0;

    public static final double kS = 0.22;
    public static final double kV = 1.98;
    public static final double kA = 0.2;

    public static final SimpleMotorFeedforward swerveFF = 
      new SimpleMotorFeedforward(kS, kV, kA);

    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            // Front left
            new Translation2d(kTrackWidthMeters / 2.0, kWheelBaseMeters / 2.0),
            // Front right
            new Translation2d(kTrackWidthMeters / 2.0, -kWheelBaseMeters / 2.0),
            // Back left
            new Translation2d(-kTrackWidthMeters / 2.0, kWheelBaseMeters / 2.0),
            // Back right
            new Translation2d(-kTrackWidthMeters / 2.0, -kWheelBaseMeters / 2.0));
    }
  
  public static class Settings {
    ///////////////// ADVANTAGE KIT LOGGING /////////////////

    private static final RobotType robot = RobotType.ROBOT_2023P;

    private static final Alert invalidRobotAlert =
        new Alert("Invalid robot selected, using competition robot as default.", AlertType.ERROR);

    public static final Map<RobotType, String> logFolders =
        Map.of(RobotType.ROBOT_2023P, "/media/sda2/");

    public static enum Mode {
      REAL,
      REPLAY,
      SIM
    }

    public static enum RobotType {
      ROBOT_2023C,
      ROBOT_2023P,
      ROBOT_SIMBOT
    }

    public static RobotType getRobot() {
      if (RobotBase.isReal()) {
        if (robot == RobotType.ROBOT_SIMBOT) { // Invalid robot selected
          invalidRobotAlert.set(true);
          return RobotType.ROBOT_2023C;
        } else {
          return robot;
        }
      } else {
        return robot;
      }
    }

    public static Mode getMode() {
      switch (getRobot()) {
        case ROBOT_2023C:
        case ROBOT_2023P:
          return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

        case ROBOT_SIMBOT:
          return Mode.SIM;

        default:
          return Mode.REAL;
      }
    }
  }
}
