package com.rambots4571.chargedup.robot;

import com.rambots4571.rampage.math.Converter;
import com.rambots4571.rampage.swerve.CTREModuleState;
import com.rambots4571.rampage.swerve.SwerveModuleConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import com.rambots4571.chargedup.robot.Constants.DriveConstants;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
  public int moduleNumber;
  private Rotation2d angleOffset;
  private Rotation2d lastAngle;

  private TalonFX turnMotor;
  private TalonFX driveMotor;
  private CANCoder angleEncoder;

  SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          DriveConstants.driveKS, DriveConstants.driveKV, DriveConstants.driveKA);

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    this.angleOffset = moduleConstants.getAngleOffset();

    /* Angle Encoder Config */
    angleEncoder = new CANCoder(moduleConstants.getCancoderID(), "BOYSALIAR");
    configAngleEncoder();

    /* Angle Motor Config */
    turnMotor = new TalonFX(moduleConstants.getAngleMotorID(), "BOYSALIAR");
    configAngleMotor();

    /* Drive Motor Config */
    driveMotor = new TalonFX(moduleConstants.getDriveMotorID(), "BOYSALIAR");
    configDriveMotor();

    lastAngle = getState().angle;
  }

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
              DriveConstants.driveGearRatio);
      driveMotor.set(
          ControlMode.Velocity,
          velocity,
          DemandType.ArbitraryFeedForward,
          feedforward.calculate(desiredState.speedMetersPerSecond));
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
        Converter.degreesToFalcon(angle.getDegrees(), DriveConstants.angleGearRatio));
    lastAngle = angle;
  }

  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(
        Converter.falconToDegrees(
            turnMotor.getSelectedSensorPosition(), DriveConstants.angleGearRatio));
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
  }

  public void resetToAbsolute() {
    double absolutePosition =
        Converter.degreesToFalcon(
            getCanCoder().getDegrees() - angleOffset.getDegrees(), DriveConstants.angleGearRatio);
    turnMotor.setSelectedSensorPosition(absolutePosition);
  }

  private void configAngleEncoder() {
    angleEncoder.configFactoryDefault();
    angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
  }

  private void configAngleMotor() {
    turnMotor.configFactoryDefault();
    turnMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
    turnMotor.setInverted(DriveConstants.angleMotorInvert);
    turnMotor.setNeutralMode(DriveConstants.angleNeutralMode);
    turnMotor.configNeutralDeadband(0.1);
    resetToAbsolute();
  }

  private void configDriveMotor() {
    driveMotor.configFactoryDefault();
    driveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
    driveMotor.setInverted(DriveConstants.driveMotorInvert);
    driveMotor.setNeutralMode(DriveConstants.driveNeutralMode);
    driveMotor.configNeutralDeadband(0.1);
    driveMotor.setSelectedSensorPosition(0);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        Converter.falconToMPS(
            driveMotor.getSelectedSensorVelocity(),
            DriveConstants.kWheelCircumference,
            DriveConstants.driveGearRatio),
        getAngle());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        Converter.falconToMeters(
            driveMotor.getSelectedSensorPosition(),
            DriveConstants.kWheelCircumference,
            DriveConstants.driveGearRatio),
        getAngle());
  }

  public void stopMotors() {
    turnMotor.set(ControlMode.PercentOutput, 0);
    driveMotor.set(ControlMode.PercentOutput, 0);
  }
}
