package com.rambots4571.chargedup.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.rambots4571.chargedup.robot.Constants.DriveConstants;
import com.rambots4571.chargedup.robot.Constants.Settings;
import com.rambots4571.chargedup.robot.subsystems.DriveTrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDriveCommand extends CommandBase {
    
    private DriveTrain driveTrain;

    private DoubleSupplier translationSupplier, strafeSupplier, rotationSupplier;
    private BooleanSupplier robotCentricSupplier;

    public SwerveDriveCommand(DriveTrain driveTrain, DoubleSupplier translationSupplier, DoubleSupplier strafeSupplier, DoubleSupplier rotationSupplier, BooleanSupplier robotCentricSupplier) {
        this.driveTrain = driveTrain;
        addRequirements(driveTrain);

        this.translationSupplier = translationSupplier;
        this.strafeSupplier = strafeSupplier;;
        this.rotationSupplier = rotationSupplier;
        this.robotCentricSupplier = robotCentricSupplier;
    }

    @Override
    public void execute() {
        double translationVal = MathUtil.applyDeadband(translationSupplier.getAsDouble(), Settings.STICK_DEADBAND);
        double strafeVal = MathUtil.applyDeadband(strafeSupplier.getAsDouble(), Settings.STICK_DEADBAND);
        double rotationVal = MathUtil.applyDeadband(rotationSupplier.getAsDouble(), Settings.STICK_DEADBAND);

        driveTrain.drive(new Translation2d(translationVal, strafeVal).times(DriveConstants.kMaxSpeedMetersPerSecond), 
            (rotationVal * DriveConstants.kMaxAngularSpeedMetersPerSecond), robotCentricSupplier.getAsBoolean(), true);
    }

}
