package com.rambots4571.chargedup.robot;

import com.rambots4571.rampage.controller.Gamepad;
import com.rambots4571.rampage.controller.PS4Controller;
import com.rambots4571.rampage.controller.PS4Controller.Button;

import com.pathplanner.lib.auto.SwerveAutoBuilder;

import com.rambots4571.chargedup.robot.Constants.DriveConstants;
import com.rambots4571.chargedup.robot.Constants.Settings;
import com.rambots4571.chargedup.robot.commands.SwerveDriveCommand;
import com.rambots4571.chargedup.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  // Joysticks
  public final PS4Controller driverController = new PS4Controller(Settings.DRIVERCONTROLLER);
  public final Gamepad gamepad = new Gamepad(Settings.GAMEPAD);

  public final Trigger robotCentricToggle = driverController.getButton(Button.Circle);

  // Misceallaneous
  public final SwerveAutoBuilder autoBuilder;

  public final SendableChooser<Command> autonChooser = new SendableChooser<>();

  // Subsystems
  private final DriveTrain driveTrain = new DriveTrain();

  // Commands
  private final SwerveDriveCommand swerveDriveCommand;

  public RobotContainer() {
    // TODO: Controllers/Axis might be a failure point
    swerveDriveCommand =
        new SwerveDriveCommand(
            driveTrain,
            () -> -driverController.getAxisValue(PS4Controller.Axis.LeftY),
            () -> -driverController.getAxisValue(PS4Controller.Axis.LeftX),
            () -> -driverController.getAxisValue(PS4Controller.Axis.RightX),
            () -> robotCentricToggle.getAsBoolean());

    driveTrain.setDefaultCommand(swerveDriveCommand);

    // Other Such Stuff
    autoBuilder =
        new SwerveAutoBuilder(
            driveTrain::getPose,
            driveTrain::resetOdometry,
            DriveConstants.kDriveKinematics,
            DriveConstants.tranlationPID,
            DriveConstants.rotationPID,
            driveTrain::setModuleStates,
            Settings.eventMap,
            driveTrain);

    autonChooser.addOption("Bare Wasteman", null);

    SmartDashboard.putData("Auton Chooser", autonChooser);

    // Shitposts written by : Karol K from team 3017 The Patriots
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // (Driver) Y -> Reset Gyro
    driverController.getButton(Button.Triangle).onTrue(zeroGyro());
  }

  private Command zeroGyro() {
    return new InstantCommand(driveTrain::zeroGyro, driveTrain);
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
