package com.rambots4571.chargedup.robot.commands.elevator;

import com.rambots4571.chargedup.robot.subsystems.Elevator;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;

public class TestElevatorCommand extends CommandBase {
  private final Elevator elevator;
  private final DoubleSupplier motorSpeed;
  private double currVel;
  private double maxVel;
  private double maxAccel;

  public TestElevatorCommand(Elevator elevator, DoubleSupplier motorSpeed) {
    this.elevator = elevator;
    this.motorSpeed = motorSpeed;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    currVel = 0;
    maxVel = 0;
    maxAccel = 0;
  }

  private void updateValues() {
    double vel = elevator.getRawSpeed();
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
    elevator.setBaseMotor(motorSpeed.getAsDouble());
    updateValues();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    elevator.stopMotors();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("raw positon", elevator::getRawEncoderPosition, null);
    builder.addDoubleProperty("max velocity", () -> maxVel, null);
    builder.addDoubleProperty("velocity", () -> currVel, null);
    builder.addDoubleProperty("max acceleration", () -> maxAccel, null);
  }
}