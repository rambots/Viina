package com.rambots4571.chargedup.robot.commands.arm;

import com.rambots4571.chargedup.robot.subsystems.Arm;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;

public class TestArmCommand extends CommandBase {

  private final Arm arm;
  private final DoubleSupplier motorSpeed;

  private double currVel;
  private double maxVel;
  private double maxAccel;

  public TestArmCommand(Arm arm, DoubleSupplier motorSpeed) {
    this.arm = arm;
    this.motorSpeed = motorSpeed;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    currVel = 0;
    maxVel = 0;
    maxAccel = 0;
  }

  public void updateValues() {
    double vel = arm.getRawSpeed();
    // updates the max recorded vel
    maxVel = vel > maxVel ? vel : maxVel;
    // calculate acceleration discretely
    double currAccel = (vel - currVel) / 0.02; // u/100ms/s
    // updates the max recorded acceleration
    maxAccel = currAccel > maxAccel ? currAccel : maxAccel;

    currVel = vel;
  }

  @Override
  public void execute() {
    arm.setPower(motorSpeed.getAsDouble());
    updateValues();
  }

  @Override
  public void end(boolean interrupted) {
    arm.stopArm();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("raw positon", arm::getRawEncoderPosition, null);
    builder.addDoubleProperty("max velocity", () -> maxVel, null);
    builder.addDoubleProperty("velocity", () -> currVel, null);
    builder.addDoubleProperty("max acceleration", () -> maxAccel, null);
  }
}