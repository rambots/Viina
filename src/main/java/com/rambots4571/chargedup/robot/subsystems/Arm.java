package com.rambots4571.chargedup.robot.subsystems;

import com.rambots4571.rampage.motor.TalonPID;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.rambots4571.chargedup.robot.Constants.ArmConstants;
import com.rambots4571.chargedup.robot.Constants.ArmConstants.pivotPIDF;
import com.rambots4571.chargedup.robot.Constants.Settings;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

  private final WPI_TalonFX armMotor, pivotMotor;
  private final List<WPI_TalonFX> allMotors;

  private final TalonPID armMotorController, pivotMotorController;
  private double desiredLength, desiredAngle;

  private static Arm instance = new Arm();

  public static Arm getInstance() {
    return instance;
  }

  public Arm() {
    armMotor = new WPI_TalonFX(ArmConstants.ARM_MOTOR, "BOYSALIAR");
    pivotMotor = new WPI_TalonFX(ArmConstants.PIVOT_MOTOR, "BOYSALIAR");

    allMotors = Arrays.asList(armMotor, pivotMotor);

    armMotorController = new TalonPID(armMotor);
    armMotorController.setPIDF(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD, ArmConstants.kF);

    pivotMotorController = new TalonPID(pivotMotor);
    pivotMotorController.setPIDF(pivotPIDF.kP, pivotPIDF.kI, pivotPIDF.kD, pivotPIDF.kF);

    addChild("Arm Motor PID", armMotorController.getTuner());
    addChild("Pivot Motor PID", pivotMotorController.getTuner());

    configMotors();

    configMotionMagic();
  }

  public void configMotors() {
    allMotors.forEach(motor -> {
        motor.configFactoryDefault();

        motor.setInverted(ArmConstants.INVERT);
        motor.setNeutralMode(ArmConstants.MODE);

        motor.configSupplyCurrentLimit(ArmConstants.SUPPLY_LIMIT);
        motor.configStatorCurrentLimit(ArmConstants.STATOR_LIMIT);

        motor.configOpenloopRamp(ArmConstants.RAMP_RATE, Settings.timeoutMs);

        motor.enableVoltageCompensation(true);
        motor.configVoltageCompSaturation(12, Settings.timeoutMs);
    });
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

  // *****************************************
  // ************** Arm Motor ****************
  // *****************************************

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

  public void stopArm() {
    armMotor.set(0);
  }

  public double getRawSpeed() {
    return armMotor.getSelectedSensorVelocity();
  }

  public double getRawEncoderPosition() {
    return armMotor.getSelectedSensorPosition();
  }

  // *****************************************
  // **************** Pivot ******************
  // *****************************************

  /**
   * Set the pivot of the arm
   *
   * @param raw encoder ticks
   */
  public void setAngle(double raw) {
    pivotMotor.set(ControlMode.Position, raw);
  }

  /**
   * Updates the desired angle which is set inside periodic()`
   *
   * @param desiredLength
   */
  public void setDesiredAngle(double desiredAngle) {
    this.desiredAngle = desiredAngle;
  }

  // For testing purposes only
  public void setPivot(double speed) {
    pivotMotor.set(speed * 0.3);
  }

  public void stopPivot() {
    pivotMotor.set(0);
  }

  @Override
  public void periodic() {
    // setLength(desiredLength);
    // setAngle(desiredAngle)
  }
}
