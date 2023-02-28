package com.rambots4571.chargedup.robot.commands;

import com.rambots4571.chargedup.robot.Constants;
import com.rambots4571.chargedup.robot.Constants.DriveConstants;
import com.rambots4571.chargedup.robot.subsystems.DriveTrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class SwerveDriveCommand extends CommandBase {

  private DriveTrain driveTrain;

  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;

  private BooleanSupplier robotCentricSup;

  public SwerveDriveCommand(
      DriveTrain driveTrain,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      BooleanSupplier robotCentricSup) {
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;
  }

  @Override
  public void execute() {
    double translationVal =
        MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
    double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
    double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

    driveTrain.drive(
        new Translation2d(translationVal, strafeVal).times(DriveConstants.kMaxSpeedMetersPerSecond),
        rotationVal * DriveConstants.kMaxAngularSpeedRadiansPerSecond,
        !robotCentricSup.getAsBoolean(),
        true);
  }
}
