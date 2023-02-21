package com.rambots4571.chargedup.robot.utils;

import com.rambots4571.rampage.math.Converter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.rambots4571.chargedup.robot.Constants.DriveConstants;
import com.rambots4571.chargedup.robot.Robot;
import com.rambots4571.chargedup.robot.swerve.CTREModuleState;
import com.rambots4571.chargedup.robot.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {

  public int moduleNumber;
  private Rotation2d lastAngle;
  private Rotation2d angleOffset;

  private TalonFX driveMotor;
  private TalonFX turnMotor;
  private CANCoder angleEncoder;

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    this.angleOffset = moduleConstants.angleOffset;

    // Configs
    angleEncoder = new CANCoder(moduleConstants.cancoderID, "BOYSALIAR");
    configAngleEncoder();

    turnMotor = new TalonFX(moduleConstants.angleMotorID, "BOYSALIAR");
    configTurnMotor();

    driveMotor = new TalonFX(moduleConstants.driveMotorID, "BOYSALIAR");
    configDriveMotor();

    lastAngle = getState().angle;
  }

  // *****************************************
  // ************* MODULE STUFF **************
  // *****************************************

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState = CTREModuleState.optimize(desiredState, getState().angle);

    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput =
          desiredState.speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond;
      driveMotor.set(ControlMode.PercentOutput, percentOutput);
    } else {
      double velocity =
          Converter.MPSToFalcon(
              desiredState.speedMetersPerSecond,
              DriveConstants.kWheelCircumference,
              DriveConstants.kDriveGearRatio);
      driveMotor.set(
          ControlMode.Velocity,
          velocity,
          DemandType.ArbitraryFeedForward,
          DriveConstants.swerveFF.calculate(desiredState.speedMetersPerSecond));
    }
  }

  private void setAngle(SwerveModuleState desiredState) {
    Rotation2d angle =
        (Math.abs(desiredState.speedMetersPerSecond)
                <= (DriveConstants.kMaxSpeedMetersPerSecond * 0.01))
            ? lastAngle
            : desiredState.angle;

    turnMotor.set(
        ControlMode.Position,
        Converter.degreesToFalcon(
            turnMotor.getSelectedSensorPosition(), DriveConstants.kTurnGrearRatio));
    lastAngle = angle;
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        Converter.falconToMPS(
            driveMotor.getSelectedSensorVelocity(),
            DriveConstants.kWheelCircumference,
            DriveConstants.kDriveGearRatio),
        getAngle());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        Converter.falconToMeters(
            driveMotor.getSelectedSensorPosition(),
            DriveConstants.kWheelCircumference,
            DriveConstants.kDriveGearRatio),
        getAngle());
  }

  public void stopMotors() {
    turnMotor.set(ControlMode.PercentOutput, 0);
    driveMotor.set(ControlMode.PercentOutput, 0);
  }

  // *****************************************
  // ************* ANGLE STUFF ***************
  // *****************************************

  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(
        Converter.falconToDegrees(
            turnMotor.getSelectedSensorPosition(), DriveConstants.kTurnGrearRatio));
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
  }

  public void resetToAbsolute() {
    double absolutePosition =
        Converter.degreesToFalcon(
            (getCanCoder().getDegrees() - angleOffset.getDegrees()),
            DriveConstants.kTurnGrearRatio);
    turnMotor.setSelectedSensorPosition(absolutePosition);
  }

  // *****************************************
  // ************* CONFIGS *******************
  // *****************************************

  private void configAngleEncoder() {
    angleEncoder.configFactoryDefault();
    angleEncoder.configAllSettings(Robot.configs.canCoderConfig);
  }

  private void configTurnMotor() {
    turnMotor.configFactoryDefault(100);
    turnMotor.configAllSettings(Robot.configs.turnFXConfig);
    turnMotor.setInverted(DriveConstants.turnMotorInvert);
    turnMotor.setNeutralMode(DriveConstants.angleNeutralMode);
    resetToAbsolute();
  }

  private void configDriveMotor() {
    driveMotor.configFactoryDefault();
    driveMotor.configAllSettings(Robot.configs.driveFXConfig);
    driveMotor.setInverted(DriveConstants.driveMotorInvert);
    driveMotor.setNeutralMode(DriveConstants.driveNeutralMode);
    driveMotor.setSelectedSensorPosition(0);
  }
}
