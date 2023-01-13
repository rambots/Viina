// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.rambots4571.chargedup.robot;

import com.rambots4571.chargedup.robot.utils.Alert;
import com.rambots4571.chargedup.robot.utils.Alert.AlertType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.Map;

public final class Constants {

  public static class DriveConstants {
    ///////////////// CAN IDs /////////////////
    // TODO: Find steer offsets
    public static final int FRONT_LEFT_DRIVE_MOTOR = 1;
    public static final int FRONT_LEFT_STEER_MOTOR = 2;
    public static final int FRONT_LEFT_STEER_ENCODER = 3;
    public static final double FRONT_LEFT_STEER_OFFSET = 0;

    public static final int FRONT_RIGHT_DRIVE_MOTOR = 4;
    public static final int FRONT_RIGHT_STEER_MOTOR = 5;
    public static final int FRONT_RIGHT_STEER_ENCODER = 6;
    public static final double FRONT_RIGHT_STEER_OFFSET = 0;

    public static final int BACK_LEFT_DRIVE_MOTOR = 7;
    public static final int BACK_LEFT_STEER_MOTOR = 8;
    public static final int BACK_LEFT_STEER_ENCODER = 9;
    public static final double BACK_LEFT_STEER_OFFSET = 0;

    public static final int BACK_RIGHT_DRIVE_MOTOR = 10;
    public static final int BACK_RIGHT_STEER_MOTOR = 11;
    public static final int BACK_RIGHT_STEER_ENCODER = 12;
    public static final double BACK_RIGHT_STEER_OFFSET = 0;

    public static final int PIGEON_IMU_2 = 13;

    ///////////////// DRIVETRAIN CHARACTERISTICS /////////////////
    public static final double kTrackWidthMeters = 0.42;
    public static final double kWheelBaseMeters = 0.73;

    public static final double kMaxSpeedMetersPerSecond = 3.0;

    public static final double kMaxVoltage = 12.0;

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
