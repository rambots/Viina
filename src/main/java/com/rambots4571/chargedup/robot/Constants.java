package com.rambots4571.chargedup.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;

import com.rambots4571.chargedup.robot.lib.util.COTSFalconSwerveConstants;
import com.rambots4571.chargedup.robot.lib.util.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.HashMap;
import java.util.List;

public final class Constants {

  public static final class DriveConstants {
    ///////////////// CAN IDs /////////////////

    /* Front Left Module - Module 0 */
    public static final class Mod0 { 
        public static final int driveMotorID = 2;
        public static final int angleMotorID = 3;
        public static final int canCoderID = 4;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(353.8);
        public static final SwerveModuleConstants constants =
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }
  
      /* Front Right Module - Module 1 */
      public static final class Mod1 { 
        public static final int driveMotorID = 5;
        public static final int angleMotorID = 6;
        public static final int canCoderID = 7;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(23.6);
        public static final SwerveModuleConstants constants =
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }
  
      /* Back Left Module - Module 2 */
      public static final class Mod2 { 
        public static final int driveMotorID = 8;
        public static final int angleMotorID = 9;
        public static final int canCoderID = 10;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(235);
        public static final SwerveModuleConstants constants =
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }
  
      /* Back Right Module - Module 3 */
      public static final class Mod3 { 
        public static final int driveMotorID = 11;
        public static final int angleMotorID = 12;
        public static final int canCoderID = 13;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(349.5);
        public static final SwerveModuleConstants constants =
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }

    public static final int pigeonID = 14;

    ///////////////// PHYSICAL CHARACTERISTICS /////////////////
    public static final COTSFalconSwerveConstants chosenModule = 
        COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

    public static final double kTrackWidth =
        Units.inchesToMeters(27); 
    public static final double kWheelBase =
        Units.inchesToMeters(27); 

    public static final double kWheelCircumference = chosenModule.wheelCircumference;

    /* Swerve Kinematics */
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
            new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
            new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
            new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0));

    /* Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    /* Motor Inverts */
    public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
    public static final boolean driveMotorInvert = true;
    
    /* Gyro Invert */
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = chosenModule.canCoderInvert;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 25;
    public static final int anglePeakCurrentLimit = 40;
    public static final double anglePeakCurrentDuration = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveContinuousCurrentLimit = 35;
    public static final int drivePeakCurrentLimit = 60;
    public static final double drivePeakCurrentDuration = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    ///////////////// CONTROL CHARACTERISTICS /////////////////
    // TODO: Tune ramprate
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.16;
    public static final double angleKI = chosenModule.angleKI;
    public static final double angleKD = chosenModule.angleKD;
    public static final double angleKF = chosenModule.angleKF;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.15; // TODO: Tuned but further tuning is better
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /* Drive Motor Characterization Values
     * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
    public static final double driveKS = (0.32 / 12); // TODO: This must be tuned to specific robot
    public static final double driveKV = (1.51 / 12);
    public static final double driveKA = (0.27 / 12);

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double kMaxSpeedMetersPerSecond = 4.5; // TODO: This must be tuned to specific robot
    /** Radians per Second */
    public static final double kMaxAngularSpeedRadiansPerSecond =
        10.0; // TODO: This must be tuned to specific robot

    /* Neutral Modes */
    public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
    public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

    public static final PIDConstants tranlationPID = new PIDConstants(driveKP, driveKI, driveKD);
    public static final PIDConstants rotationPID = new PIDConstants(angleKP, angleKI, angleKD);
  }

  public static final class Settings {

    ///////////////// JOYSTICKS /////////////////

    public static final int DRIVERCONTROLLER = 3;
    public static final int GAMEPAD = 2;
    public static final double STICK_DEADBAND = 0.1;

    ///////////////// PHOTONVISION /////////////////
    //TODO: Get real values for all ts
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
    public static final List<PathPlannerTrajectory> Test = PathPlanner.loadPathGroup("Test", 3, 4);
    public static HashMap<String, Command> eventMap = new HashMap<String, Command>();
  }
}
