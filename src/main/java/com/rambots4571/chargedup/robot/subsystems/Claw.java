package com.rambots4571.chargedup.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.rambots4571.chargedup.robot.Constants.ClawConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {

  private final CANSparkMax wristMotor;

  private static Claw instance = new Claw();

  public static Claw getInstance() {
    return instance;
  }

  public Claw() {
    wristMotor = new CANSparkMax(ClawConstants.WRIST_MOTOR, MotorType.kBrushless);

    wristMotor.restoreFactoryDefaults();
  }

  public void setSpeed(double speed) {
    wristMotor.set(speed);
  }

  public void stop() {
    wristMotor.set(0);
  }
}
