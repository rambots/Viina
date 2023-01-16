package com.rambots4571.chargedup.robot.utils;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import com.rambots4571.chargedup.robot.Constants.DriveConstants;
import com.rambots4571.chargedup.robot.Constants.Settings;
import com.rambots4571.chargedup.robot.Robot;
import com.rambots4571.rampage.math.Converter;
import com.rambots4571.rampage.swerve.CTREModuleState;
import com.rambots4571.rampage.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;

public class SwerveModule {

  public int moduleNumber;
  private Rotation2d lastAngle;
  private Rotation2d angleOffset;

  private TalonFX driveMotor;
  private TalonFX turnMotor;
  private CANCoder angleEncoder;

  public double CANcoderInitTime = 0.0;

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    this.angleOffset = moduleConstants.getAngleOffset();

    // Configs
    driveMotor = new TalonFX(moduleConstants.getDriveMotorID());

    turnMotor = new TalonFX(moduleConstants.getAngleMotorID());

    angleEncoder = new CANCoder(moduleConstants.getCancoderID());

    lastAngle = getState().angle;

    configDriveMotor();
    configTurnMotor();
    configAngleEncoder();
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

  private void waitForCanCoder() {
    /*
     * Wait for up to 1000 ms for a good CANcoder signal.
     *
     * This prevents a race condition during program startup
     * where we try to synchronize the Falcon encoder to the
     * CANcoder before we have received any position signal
     * from the CANcoder.
     */
    for (int i = 0; i < 100; ++i) {
      angleEncoder.getAbsolutePosition();
      if (angleEncoder.getLastError() == ErrorCode.OK) {
        break;
      }
      Timer.delay(0.010);
      CANcoderInitTime += 10;
    }
  }

  private void resetToAbsolute() {
    waitForCanCoder();

    double absolutePosition =
        Converter.degreesToFalcon(
            (getCanCoder().getDegrees() - angleOffset.getDegrees()),
            DriveConstants.kTurnGrearRatio);
    turnMotor.setSelectedSensorPosition(absolutePosition);
  }

  // *****************************************
  // ************* CONFIGS *******************
  // *****************************************

  private void configDriveMotor() {
    driveMotor.configFactoryDefault();
    driveMotor.configAllSettings(Robot.configs.driveFXConfig, Settings.timeoutMs);
    driveMotor.setInverted(DriveConstants.driveMotorInvert);
    driveMotor.setNeutralMode(DriveConstants.driveNeutralMode);
    driveMotor.setSelectedSensorPosition(0);
  }

  private void configTurnMotor() {
    turnMotor.configFactoryDefault();
    turnMotor.configAllSettings(Robot.configs.turnFXConfig, Settings.timeoutMs);
    turnMotor.setInverted(DriveConstants.turnMotorInvert);
    turnMotor.setNeutralMode(DriveConstants.angleNeutralMode);
    resetToAbsolute();
  }

  private void configAngleEncoder() {
    angleEncoder.configFactoryDefault();
    angleEncoder.configAllSettings(Robot.configs.canCoderConfig, Settings.timeoutMs);
  }
}
