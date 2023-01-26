package com.rambots4571.chargedup.robot.utils;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import com.rambots4571.chargedup.robot.Constants.DriveConstants;

public class CTREConfigs {

  public TalonFXConfiguration driveFXConfig, turnFXConfig;
  public CANCoderConfiguration canCoderConfig;

  public SupplyCurrentLimitConfiguration angleCurrentLimit, driveCurrentLimit;

  public CTREConfigs() {
    turnFXConfig = new TalonFXConfiguration();
    driveFXConfig = new TalonFXConfiguration();

    canCoderConfig = new CANCoderConfiguration();

    // Turn Motor Configs
    angleCurrentLimit =
        new SupplyCurrentLimitConfiguration(
            DriveConstants.angleEnableCurrentLimit,
            DriveConstants.angleContinuousCurrentLimit,
            DriveConstants.anglePeakCurrentLimit,
            DriveConstants.anglePeakCurrentDuration);

    turnFXConfig.slot0.kP = DriveConstants.angleKP;
    turnFXConfig.slot0.kI = DriveConstants.angleKI;
    turnFXConfig.slot0.kD = DriveConstants.angleKD;
    turnFXConfig.slot0.kF = DriveConstants.angleKF;
    turnFXConfig.supplyCurrLimit = angleCurrentLimit;

    // Drive Motor Configs
    driveCurrentLimit =
        new SupplyCurrentLimitConfiguration(
            DriveConstants.driveEnableCurrentLimit,
            DriveConstants.driveContinuousCurrentLimit,
            DriveConstants.drivePeakCurrentLimit,
            DriveConstants.drivePeakCurrentDuration);

    driveFXConfig.slot0.kP = DriveConstants.driveKP;
    driveFXConfig.slot0.kI = DriveConstants.driveKI;
    driveFXConfig.slot0.kD = DriveConstants.driveKD;
    driveFXConfig.slot0.kF = DriveConstants.driveKF;
    driveFXConfig.supplyCurrLimit = driveCurrentLimit;
    driveFXConfig.openloopRamp = DriveConstants.openLoopRamp;
    driveFXConfig.closedloopRamp = DriveConstants.closedLoopRamp;

    // CANCoder Configs
    canCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    canCoderConfig.sensorDirection = DriveConstants.canCoderInvert;
    canCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    canCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
  }
}
