package com.rambots4571.chargedup.robot.commands.drive;

import com.rambots4571.chargedup.robot.subsystems.DriveTrain;
import com.rambots4571.chargedup.robot.utils.PhotonVision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoAlign extends CommandBase {

  private final DriveTrain driveTrain;
  private final PIDController rotationController;
  private final PhotonVision vision = PhotonVision.getInstance();

  private final double rotationKP = 0;
  private final double rotationKI = 0;
  private final double rotationKD = 0;

  public AutoAlign(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);

    rotationController = new PIDController(rotationKP, rotationKI, rotationKD);
    rotationController.setTolerance(2.0);
  }

  @Override
  public void initialize() {
    if (vision.hasValidTarget()) rotationController.setSetpoint(0);
  }

  @Override
  public void execute() {
    if (vision.hasValidTarget()) {

      double xOffset = vision.getBestTarget().getYaw();
      double rotationSpeed = -rotationController.calculate(xOffset, 0);

      driveTrain.drive(null, rotationSpeed, false, false);
    }
  }

  @Override
  public void end(boolean interrupted) {
    driveTrain.stopMotors();
  }

  @Override
  public boolean isFinished() {
    return !vision.hasValidTarget() || rotationController.atSetpoint();
  }
}
