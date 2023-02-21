// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.rambots4571.chargedup.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;

import com.rambots4571.chargedup.robot.swerve.COTSFalconSwerveConstants;
import com.rambots4571.chargedup.robot.swerve.SwerveModuleConstants;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.HashMap;
import java.util.List;
import lombok.AllArgsConstructor;
import lombok.Getter;

public final class Constants {

  public static class DriveConstants {
    ///////////////// CAN IDs /////////////////

    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 2;
      public static final int angleMotorID = 3;
      public static final int canCoderID = 4;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(175.78);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 5;
      public static final int angleMotorID = 6;
      public static final int canCoderID = 7;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(194.77);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 8;
      public static final int angleMotorID = 9;
      public static final int canCoderID = 10;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(55.02);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 11;
      public static final int angleMotorID = 12;
      public static final int canCoderID = 13;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(164.44);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    public static final int PIGEON_IMU_2 = 14;

    ///////////////// PHYSICAL CHARACTERISTICS /////////////////
    public static final COTSFalconSwerveConstants chosenModule =
        COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

    // Inversions
    public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;
    public static final boolean turnMotorInvert = chosenModule.angleMotorInvert;

    public static final boolean canCoderInvert = chosenModule.canCoderInvert;

    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    // Neutral Modes
    public static final NeutralMode driveNeutralMode = NeutralMode.Brake;
    public static final NeutralMode angleNeutralMode = NeutralMode.Coast;

    // Swerve Current Limiting
    public static final int angleContinuousCurrentLimit = 25;
    public static final int anglePeakCurrentLimit = 40;
    public static final double anglePeakCurrentDuration = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveContinuousCurrentLimit = 35;
    public static final int drivePeakCurrentLimit = 60;
    public static final double drivePeakCurrentDuration = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    public static final double kTrackWidthMeters = Units.inchesToMeters(21.75);
    public static final double kWheelBaseMeters = Units.inchesToMeters(21.75);
    public static final double kWheelCircumference = chosenModule.wheelCircumference;

    public static final double kDriveGearRatio = chosenModule.driveGearRatio;
    public static final double kTurnGrearRatio = chosenModule.angleGearRatio;

    public static final double kMaxSpeedMetersPerSecond = 3.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3.5;
    public static final double kMaxAngularSpeedMetersPerSecond = 4.0;

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

    ///////////////// CONTROL CHARACTERISTICS /////////////////
    public static final double angleKP = chosenModule.angleKP;
    public static final double angleKI = chosenModule.angleKI;
    public static final double angleKD = chosenModule.angleKD;
    public static final double angleKF = chosenModule.angleKF;

    public static final double driveKP = 0.05; // TODO: Tune this
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    // Divide SysId results by 12 to convert from volts to CTRE percentOutput
    public static final double kS = (0.32 / 12);
    public static final double kV = (1.51 / 12);
    public static final double kA = (0.27 / 12);

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final SimpleMotorFeedforward swerveFF = new SimpleMotorFeedforward(kS, kV, kA);

    public static final PIDConstants tranlationPID = new PIDConstants(driveKP, driveKI, driveKD);

    public static final PIDConstants rotationPID = new PIDConstants(angleKP, angleKI, angleKD);
  }

  public static final class Cvator {
    public static final int BASE_MOTOR_MASTER = 15;
    public static final int BASE_MOTOR_FOLLOWER = 16;

    public static final int LIMITSWITCH = 0;

    public static final double rampRate = 0.15;

    public static final NeutralMode MODE = NeutralMode.Brake;

    public static final TalonFXInvertType masterInvert = TalonFXInvertType.Clockwise;
    public static final TalonFXInvertType followerInvert = TalonFXInvertType.CounterClockwise;

    // TODO: Tune Stator/Supply Limits
    public static final StatorCurrentLimitConfiguration statorLimit =
        new StatorCurrentLimitConfiguration(true, 40, 70, 2);

    public static final SupplyCurrentLimitConfiguration supplyLimit =
        new SupplyCurrentLimitConfiguration(true, 40, 60, 4);

    // TODO: Find and tune real values
    public static final class ElevatorPIDF {

      public static final double kP = 0;
      public static final double kI = 0;
      public static final double kD = 0;
      public static final double kF = 0;
    }

    public static final class ArmPIDF {

      public static final double kP = 0;
      public static final double kI = 0;
      public static final double kD = 0;
      public static final double kF = 0;
    }

    public static final double cruiseVel = 0; // u/100_ms
    public static final double motionAccel = 0; // u/100_ms/s

    @Getter
    @AllArgsConstructor
    public static enum Position {
      BOTTOM(0, 0, 0),
      MIDDLE(0, 0, 0),
      TOP(0, 0, 0);
      private final double cubeHeight, coneHeight, armlength;
    }

    public static enum PositionMode {
      CONE,
      CUBE
    }
  }

  public static class ArmConstants {
    public static final int ARM_MOTOR = 17;

    public static final TalonFXInvertType INVERT = TalonFXInvertType.Clockwise; // TODO: Test this
    public static final NeutralMode MODE = NeutralMode.Brake;

    public static final StatorCurrentLimitConfiguration STATOR_LIMIT =
        new StatorCurrentLimitConfiguration(true, 40, 70, 2);

    public static final SupplyCurrentLimitConfiguration SUPPLY_LIMIT =
        new SupplyCurrentLimitConfiguration(true, 40, 60, 4);

    public static final double RAMP_RATE = 0.15;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kF = 0;

    public static final double cruiseVel = 3.0;
    public static final double motionAccel = 4.0;
  }

  public static class Settings {
    public static final double BALANCE_BEAM_DEGREES_GOAL = 0.0;
    public static final int timeoutMs = 10;

    ///////////////// JOYSTICKS /////////////////

    public static final int DRIVERCONTROLLER = 3;
    public static final int GAMEPAD = 2;
    public static final double STICK_DEADBAND = 0.1;

    ///////////////// PHOTONVISION /////////////////

    public static final double CAMERA_HEIGHT_METERS = 0.5;
    public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(90);
    public static final double TARGET_HEIGHT_METERS = 0.5;
    public static final double GOAL_RANGE_METERS = Units.inchesToMeters(6);

    ///////////////// AUTON /////////////////

    public static final Translation2d STARTING_TRANSLATION = new Translation2d();
    public static final Rotation2d STARTING_ANGLE = new Rotation2d();

    public static final Pose2d STARTING_POSITION = new Pose2d(STARTING_TRANSLATION, STARTING_ANGLE);

    public static HashMap<String, Command> eventMap = new HashMap<String, Command>();
  }

  public static final class AutoPaths {
    public static final List<PathPlannerTrajectory> TwoCargoBot =
        PathPlanner.loadPathGroup(
            "2CargoBottomBalance",
            DriveConstants.kMaxSpeedMetersPerSecond,
            DriveConstants.kMaxAccelerationMetersPerSecondSquared);
  }
}
