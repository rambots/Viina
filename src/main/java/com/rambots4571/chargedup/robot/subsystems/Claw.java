package com.rambots4571.chargedup.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import com.rambots4571.chargedup.robot.Constants.ClawConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Arrays;
import java.util.List;

public class Claw extends SubsystemBase {

  private final CANSparkMax leftWristMotor, rightWristMotor, intakeMotor;
  private final List<CANSparkMax> wristMotors;

  private double setpoint;
  private double desiredPitch;

  private final RelativeEncoder wristEncoder;
  private final PIDController controller;

  private static Claw instance = new Claw();

  public static Claw getInstance() {
    return instance;
  }

  public Claw() {
    leftWristMotor = new CANSparkMax(ClawConstants.LEFT_WRIST_MOTOR, MotorType.kBrushless);
    rightWristMotor = new CANSparkMax(ClawConstants.RIGHT_WRIST_MOTOR, MotorType.kBrushless);
    intakeMotor = new CANSparkMax(ClawConstants.INTAKE_MOTOR, MotorType.kBrushless);

    wristMotors = Arrays.asList(leftWristMotor, rightWristMotor, intakeMotor);

    wristEncoder = leftWristMotor.getEncoder();

    controller = new PIDController(0.1, 0, 0);
    controller.setSetpoint(427.5);
    controller.setTolerance(5);

    configMotors();
  }

  public void configMotors() {
    wristMotors.forEach(
        motor -> {
          motor.restoreFactoryDefaults();

          motor.setIdleMode(ClawConstants.MODE);
          motor.setSmartCurrentLimit(ClawConstants.NEO_550_LIMIT);

          motor.burnFlash();
        });

    leftWristMotor.setInverted(ClawConstants.LEFT_WRIST_INVERSION);
    rightWristMotor.setInverted(ClawConstants.RIGHT_WRIST_INVERSION);
    intakeMotor.setInverted(ClawConstants.LEFT_WRIST_INVERSION);
  }

  public void setIntake(double speed) {
    intakeMotor.set(speed);
  }

  public void setPitch90() {
    setPitch(427.5);
}

  public double getDisplacementOutput(double displacement) {
    double currPos = wristEncoder.getPosition();
    double setpoint = currPos + displacement;
    return controller.calculate(currPos, setpoint);
  }

  public void setPitch(double rawDis) {
    double output = getDisplacementOutput(rawDis);
    leftWristMotor.set(output);
    rightWristMotor.set(output);
  }

  public void setTwist(double rawDis) {
    double output = getDisplacementOutput(rawDis);
    leftWristMotor.set(output);
    rightWristMotor.set(-output);
  }

  public void stop() {
    leftWristMotor.set(0);
    rightWristMotor.set(0);
  }
}
