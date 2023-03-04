package com.rambots4571.chargedup.robot.commands.arm;

import com.rambots4571.chargedup.robot.subsystems.Claw;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TestClaw extends CommandBase {

  private final Claw claw;

  public TestClaw(Claw claw) {
    this.claw = claw;
    addRequirements(claw);
  }

  @Override
  public void execute() {
    claw.turnToNinety();
  }

  @Override
  public void end(boolean interrupted) {
    claw.stop();
  }

  @Override
  public boolean isFinished() {
    return claw.atSetpoint();
  }
}
