package com.rambots4571.chargedup.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import com.rambots4571.chargedup.robot.Constants.ClawConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Arrays;
import java.util.List;

public class Claw extends SubsystemBase {

  private final CANSparkMax leftWristMotor, rightWristMotor, intakeMotor;
  private final List<CANSparkMax> wristMotors;

  private double setpoint;

  private final RelativeEncoder wristEncoder;
  private final SparkMaxPIDController wristPID;
  private final PIDController controller;

  private static Claw instance = new Claw();

  public static Claw getInstance() {
    return instance;
  }

  public Claw() {
    leftWristMotor = new CANSparkMax(ClawConstants.LEFT_WRIST_MOTOR, MotorType.kBrushless);
    rightWristMotor = new CANSparkMax(ClawConstants.RIGHT_WRIST_MOTOR, MotorType.kBrushless);
    intakeMotor = new CANSparkMax(24, MotorType.kBrushless);

    wristMotors = Arrays.asList(leftWristMotor, rightWristMotor);

    wristEncoder = leftWristMotor.getEncoder();

    wristPID = leftWristMotor.getPIDController();
    wristPID.setP(3);
    wristPID.setOutputRange(-1, 1);

    controller = new PIDController(0.1, 0, 0);
    controller.setSetpoint(427.5);

    configMotors();
  }

  public void configMotors() {
    wristMotors.forEach(
        motor -> {
          motor.restoreFactoryDefaults();

          motor.setIdleMode(ClawConstants.MODE);
          motor.setSmartCurrentLimit(ClawConstants.FULL_NEO_LIMIT);

          motor.setInverted(ClawConstants.WRIST_INVERSION);

          motor.burnFlash();
        });
  }

  public void setSpeed(double speed) {
    leftWristMotor.set(speed);
  }

  public void turnToNinety() {
    leftWristMotor.set(controller.calculate(wristEncoder.getPosition(), 427.5));
  }

  public void stop() {
    leftWristMotor.set(0);
  }

  public boolean atSetpoint() {
    return Math.abs(setpoint - wristEncoder.getPosition()) <= 5;
  }
}
