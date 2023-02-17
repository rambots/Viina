package com.rambots4571.chargedup.robot.subsystems;

import com.rambots4571.rampage.motor.TalonPID;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.rambots4571.chargedup.robot.Constants.ArmConstants;
import com.rambots4571.chargedup.robot.Constants.Settings;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

  private final WPI_TalonFX armMotor;
  private final TalonPID armMotorController;
  private double desiredLength;

  private static Arm instance = new Arm();

  public static Arm getInstance() {
    return instance;
  }

  public Arm() {
    armMotor = new WPI_TalonFX(ArmConstants.ARM_MOTOR);

    armMotorController = new TalonPID(armMotor);
    armMotorController.setPIDF(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD, ArmConstants.kF);

    addChild("Arm Motor PID", armMotorController.getTuner());

    configMotor();

    configMotionMagic();
  }

  public void configMotor() {
    armMotor.configFactoryDefault();

    armMotor.setInverted(ArmConstants.INVERT);
    armMotor.setNeutralMode(ArmConstants.MODE);

    armMotor.configSupplyCurrentLimit(ArmConstants.SUPPLY_LIMIT);
    armMotor.configStatorCurrentLimit(ArmConstants.STATOR_LIMIT);

    armMotor.configOpenloopRamp(ArmConstants.RAMP_RATE, Settings.timeoutMs);

    armMotor.enableVoltageCompensation(true);
    armMotor.configVoltageCompSaturation(12, Settings.timeoutMs);
  }

  public void configMotionMagic() {
    armMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Settings.timeoutMs);
    armMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Settings.timeoutMs);
    armMotor.setStatusFramePeriod(
        StatusFrameEnhanced.Status_10_MotionMagic, 10, Settings.timeoutMs);

    armMotor.configOpenloopRamp(0.15, Settings.timeoutMs);

    armMotor.configMotionCruiseVelocity(ArmConstants.cruiseVel, Settings.timeoutMs);
    armMotor.configMotionAcceleration(ArmConstants.motionAccel, Settings.timeoutMs);
  }

  /**
   * Set the length you want the arm to extend
   *
   * @param raw encoder ticks
   */
  public void setLength(double raw) {
    armMotor.set(TalonFXControlMode.MotionMagic, raw);
  }

  /**
   * Updates the desired length which is set inside periodic()`
   *
   * @param desiredLength
   */
  public void setDesiredLength(double desiredLength) {
    this.desiredLength = desiredLength;
  }

  public void setPower(double value) {
    armMotor.set(value);
  }

  public void stop() {
    armMotor.set(0);
  }

  public double getRawSpeed() {
    return armMotor.getSelectedSensorVelocity();
  }

  public double getRawEncoderPosition() {
    return armMotor.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    // setLength(desiredLength);
  }
}
