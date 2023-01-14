// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.rambots4571.chargedup.robot;

import com.rambots4571.chargedup.robot.Constants.Settings;
import com.rambots4571.chargedup.robot.commands.SwerveDriveCommand;
import com.rambots4571.chargedup.robot.subsystems.DriveTrain;
import com.rambots4571.rampage.joystick.Controller;
import com.rambots4571.rampage.joystick.Gamepad;
import com.rambots4571.rampage.joystick.Gamepad.Button;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  // Misceallaneous 
  public static final Controller<Gamepad.Button, Gamepad.Axis> driverController = Gamepad.make(Settings.DRIVERCONTROLLER);

  public static final SendableChooser<Command> autonChooser = new SendableChooser<>();

  private final Trigger robotCentricToggle = new Trigger(driverController.getButton(Button.Y));

  // Subsytems
  private final DriveTrain driveTrain;

  // Commands 
  private final SwerveDriveCommand swerveDriveCommand;

  public RobotContainer() {
    driveTrain = DriveTrain.getInstance();

    swerveDriveCommand = new SwerveDriveCommand(
      driveTrain, 
      () -> -driverController.getRawAxis(Settings.translationAxis),
      () -> -driverController.getRawAxis(Settings.strafeAxis),
      () -> -driverController.getRawAxis(Settings.rotationAxis),
      () -> robotCentricToggle.getAsBoolean()); 

    driveTrain.setDefaultCommand(swerveDriveCommand);  

    configureBindings();
  }

  private void configureBindings() {
    // (DriverController) Y -> Rest Gyro

    driverController.getButton(Button.Y).onTrue(zeroGyro());
  }

  public Command getAutonomousCommand() {

    return null;
  }

  private Command zeroGyro() {
    return new InstantCommand(driveTrain::zeroGyro, driveTrain);
  }
}
