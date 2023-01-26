package com.rambots4571.chargedup.robot.commands.Drive;

import com.rambots4571.chargedup.robot.subsystems.DriveTrain;
import com.rambots4571.chargedup.robot.utils.PhotonVision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoAlign extends CommandBase {
    
    private final DriveTrain driveTrain;
    private final PIDController translationController, rotationController;
    private final PhotonVision vision = PhotonVision.getInstance();

    private final double translationKP = 0;
    private final double translationKI = 0;
    private final double translationKD = 0;

    private final double rotationKP = 0;
    private final double rotationKI = 0;
    private final double rotationKD = 0;

    public AutoAlign(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        addRequirements(driveTrain);

        translationController = new PIDController(translationKP, translationKI, translationKD);                 

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
        double range = vision.getEstimatedDistanceToTarget();
        
        }
    }
}
