package com.rambots4571.chargedup.robot.commands.elevator;

import com.rambots4571.chargedup.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private void log() {
    SmartDashboard.putNumber("Elevator/raw position", elevator.getRawEncoderPosition());
    SmartDashboard.putNumber("Elevator/max vel", maxVel);
    SmartDashboard.putNumber("Elevator/vel", currVel);
    SmartDashboard.putNumber("Elevator/max accel", maxAccel);
    SmartDashboard.putNumber("Elevator/accel", currVel);
  }

  @Override
  public void execute() {
    elevator.setBaseMotor(motorSpeed.getAsDouble());
    updateValues();
    log();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    elevator.stopMotors();
  }
}
