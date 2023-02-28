package com.rambots4571.chargedup.robot.commands.drive;

import com.rambots4571.chargedup.robot.subsystems.DriveTrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class BalanceOnBeam extends CommandBase {

  private final DriveTrain driveTrain;
  private final PIDController forwardController;

  private final double kP = 0.1;
  private final double kI = 0.1;
  private final double kD = 0.1;

  private final double maxOutput = 0.5;

  private double currentAngle;
  private double output;

  public BalanceOnBeam(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);

    forwardController = new PIDController(kP, kI, kD);
    forwardController.setTolerance(2.0);
  }

  @Override
  public void initialize() {
    forwardController.setSetpoint(0);
  }

  @Override
  public void execute() {
    currentAngle = driveTrain.getGyroPitch();
    output = MathUtil.clamp(forwardController.calculate(currentAngle), -maxOutput, maxOutput);

    driveTrain.drive(new Translation2d(output, 0), 0, true, true);
  }

  @Override
  public void end(boolean interrupted) {
    driveTrain.stopMotors();
  }

  @Override
  public boolean isFinished() {
    return forwardController.atSetpoint();
  }
}