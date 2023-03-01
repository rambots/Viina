package com.rambots4571.chargedup.robot.utils;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import com.rambots4571.chargedup.robot.Constants.DriveConstants;

public final class CTREConfigs {
  public TalonFXConfiguration swerveAngleFXConfig;
  public TalonFXConfiguration swerveDriveFXConfig;
  public CANCoderConfiguration swerveCanCoderConfig;

  public CTREConfigs() {
    swerveAngleFXConfig = new TalonFXConfiguration();
    swerveDriveFXConfig = new TalonFXConfiguration();
    swerveCanCoderConfig = new CANCoderConfiguration();

    /* Swerve Angle Motor Configurations */
    SupplyCurrentLimitConfiguration angleSupplyLimit =
        new SupplyCurrentLimitConfiguration(
            DriveConstants.angleEnableCurrentLimit,
            DriveConstants.angleContinuousCurrentLimit,
            DriveConstants.anglePeakCurrentLimit,
            DriveConstants.anglePeakCurrentDuration);

    swerveAngleFXConfig.slot0.kP = DriveConstants.angleKP;
    swerveAngleFXConfig.slot0.kI = DriveConstants.angleKI;
    swerveAngleFXConfig.slot0.kD = DriveConstants.angleKD;
    swerveAngleFXConfig.slot0.kF = DriveConstants.angleKF;
    swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

    /* Swerve Drive Motor Configuration */
    SupplyCurrentLimitConfiguration driveSupplyLimit =
        new SupplyCurrentLimitConfiguration(
            DriveConstants.driveEnableCurrentLimit,
            DriveConstants.driveContinuousCurrentLimit,
            DriveConstants.drivePeakCurrentLimit,
            DriveConstants.drivePeakCurrentDuration);

    swerveDriveFXConfig.slot0.kP = DriveConstants.driveKP;
    swerveDriveFXConfig.slot0.kI = DriveConstants.driveKI;
    swerveDriveFXConfig.slot0.kD = DriveConstants.driveKD;
    swerveDriveFXConfig.slot0.kF = DriveConstants.driveKF;
    swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
    swerveDriveFXConfig.openloopRamp = DriveConstants.openLoopRamp;
    swerveDriveFXConfig.closedloopRamp = DriveConstants.closedLoopRamp;

    /* Swerve CANCoder Configuration */
    swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    swerveCanCoderConfig.sensorDirection = DriveConstants.canCoderInvert;
    swerveCanCoderConfig.initializationStrategy =
        SensorInitializationStrategy.BootToAbsolutePosition;
    swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
  }
}
