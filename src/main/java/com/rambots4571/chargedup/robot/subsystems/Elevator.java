package com.rambots4571.chargedup.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.rambots4571.chargedup.robot.Constants.Cvator;
import com.rambots4571.chargedup.robot.Constants.Settings;
import com.rambots4571.rampage.motor.TalonPID;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

public class Elevator extends SubsystemBase {

  private final WPI_TalonFX baseMotorMaster, baseMotorFollower;

  private final DigitalInput limitSwitch;

  private final TalonPID baseMotorController;

  private static Elevator instance = new Elevator();

  public static Elevator getInstance() {
    return instance;
  }

  private Elevator() {
    baseMotorMaster = new WPI_TalonFX(Cvator.BASE_MOTOR_MASTER);
    baseMotorFollower = new WPI_TalonFX(Cvator.BASE_MOTOR_FOLLOWER);

    Arrays.asList(baseMotorMaster, baseMotorFollower)
        .forEach(
            motor -> {
              motor.configFactoryDefault();
              motor.setNeutralMode(Cvator.MODE);
              motor.configOpenloopRamp(Cvator.rampRate, Settings.timeoutMs);
              motor.configStatorCurrentLimit(Cvator.statorLimit);
              motor.configSupplyCurrentLimit(Cvator.supplyLimit);
              motor.enableVoltageCompensation(true);
              motor.configVoltageCompSaturation(12, Settings.timeoutMs);
            });

    baseMotorMaster.setInverted(Cvator.masterInvert);
    baseMotorFollower.setInverted(Cvator.followerInvert);

    baseMotorFollower.follow(baseMotorMaster);

    baseMotorController = new TalonPID(baseMotorMaster);
    baseMotorController.setPIDF(Cvator.kP, Cvator.kI, Cvator.kD, Cvator.kF);

    addChild("BaseMotor PID", baseMotorController.getTuner());

    limitSwitch = new DigitalInput(Cvator.LIMITSWITCH);
  }

  public void configMotionMagic() {
    baseMotorMaster.configSelectedFeedbackSensor(
        FeedbackDevice.IntegratedSensor, 0, Settings.timeoutMs);
    baseMotorMaster.setStatusFramePeriod(
        StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Settings.timeoutMs);
    baseMotorMaster.setStatusFramePeriod(
        StatusFrameEnhanced.Status_10_MotionMagic, 10, Settings.timeoutMs);

    baseMotorMaster.configNominalOutputForward(0, Settings.timeoutMs);
    baseMotorMaster.configNominalOutputReverse(0, Settings.timeoutMs);
    baseMotorMaster.configPeakOutputForward(1, Settings.timeoutMs);
    baseMotorMaster.configPeakOutputReverse(-1, Settings.timeoutMs);

    baseMotorMaster.selectProfileSlot(0, 0);
    baseMotorMaster.configClosedloopRamp(0.15, Settings.timeoutMs);

    baseMotorMaster.configMotionCruiseVelocity(Cvator.cruiseVel, Settings.timeoutMs);
    baseMotorMaster.configMotionAcceleration(Cvator.motionAccel, Settings.timeoutMs);
  }

  public boolean isLimitSwitchPressed() {
    return !limitSwitch.get();
  }

  public void setBaseMotor(double speed) {
    if (isLimitSwitchPressed() && speed < 0) {
      stopMotors();
    } else {
      baseMotorMaster.set(ControlMode.PercentOutput, speed);
    }
  }

  public void stopMotors() {
    baseMotorMaster.set(0);
  }

  public void setHeight(DoubleSupplier height) {
    baseMotorMaster.set(ControlMode.MotionMagic, height.getAsDouble());
  }

  public double getRawEncoderPosition() {
    return baseMotorController.getRaw();
  }

  public double getRawSpeed() {
    return baseMotorMaster.getSelectedSensorVelocity();
  }
}
