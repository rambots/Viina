package com.rambots4571.chargedup.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.rambots4571.chargedup.robot.Constants.ArmConstants;
import com.rambots4571.chargedup.robot.Constants.Settings;
import com.rambots4571.rampage.motor.TalonPID;

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

  public void setLength(double raw) {
    armMotor.set(TalonFXControlMode.MotionMagic, raw);
  }

  public void setDesiredLength(double desiredLength) {
    this.desiredLength = desiredLength;
  }

  public void stop() {
    armMotor.set(0);
  }

  @Override
  public void periodic() {
    // setLength(desiredLength);
  }
}
