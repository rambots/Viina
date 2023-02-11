// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.rambots4571.chargedup.robot;

import com.pathplanner.lib.auto.SwerveAutoBuilder;

import com.rambots4571.chargedup.robot.Constants.AutoPaths;
import com.rambots4571.chargedup.robot.Constants.DriveConstants;
import com.rambots4571.chargedup.robot.Constants.Settings;
import com.rambots4571.chargedup.robot.commands.drive.BalanceOnBeam;
import com.rambots4571.chargedup.robot.commands.drive.SwerveDriveCommand;
import com.rambots4571.chargedup.robot.commands.elevator.TestElevatorCommand;
import com.rambots4571.chargedup.robot.subsystems.DriveTrain;
import com.rambots4571.chargedup.robot.subsystems.Elevator;
import com.rambots4571.rampage.command.RunEndCommand;
import com.rambots4571.rampage.controller.Gamepad;
import com.rambots4571.rampage.controller.Gamepad.Button;
import com.rambots4571.rampage.controller.component.DPadButton.Direction;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  // Misceallaneous
  public final Gamepad driverController = new Gamepad(Settings.DRIVERCONTROLLER);

  public final Trigger robotCentricToggle = driverController.getButton(Button.Y);

  public final SendableChooser<Command> autonChooser = new SendableChooser<>();

  public final SwerveAutoBuilder autoBuilder;

  // Subsytems
  private final DriveTrain driveTrain;
  private final Elevator elevator;

  // Commands
  private final SwerveDriveCommand swerveDriveCommand;
  private final TestElevatorCommand testElevatorCommand;
  private final BalanceOnBeam balanceOnBeam;

  public RobotContainer() {
    driveTrain = DriveTrain.getInstance();
    elevator = Elevator.getInstance();

    swerveDriveCommand =
        new SwerveDriveCommand(
            driveTrain,
            () -> driverController.getAxisValue(Gamepad.Axis.LeftYAxis),
            () -> -driverController.getAxisValue(Gamepad.Axis.LeftXAxis),
            () -> -driverController.getAxisValue(Gamepad.Axis.RightXAxis),
            () -> robotCentricToggle.getAsBoolean());

    balanceOnBeam = new BalanceOnBeam(driveTrain);

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

    Command TwoCargoBot = autoBuilder.fullAuto(AutoPaths.TwoCargoBot);

    autonChooser.addOption("Bare Wasteman", null);
    autonChooser.addOption("2 Cargo Bottom", TwoCargoBot);

    SmartDashboard.putData("Auton Chooser", autonChooser);

    testElevatorCommand =
        new TestElevatorCommand(
            Elevator.getInstance(), () -> driverController.getAxisValue(Gamepad.Axis.RightYAxis));

    // TODO: uncomment to test elevator
    // Elevator.getInstance().setDefaultCommand(testElevatorCommand);

    configureBindings();

    setEventMap();
  }

  private void setEventMap() {}

  private void configureBindings() {
    // (Driver) Y -> Reset Gyro
    driverController.getButton(Button.Y).onTrue(zeroGyro());

    // (Driver) Left DPad -> Switch Pos Mode
    driverController.getDPadButton(Direction.LEFT).onTrue(togglePositionMode());

    driverController
        .getDPadButton(Direction.UP)
        // (Driver) Top DPad -> Iterate Forward (Pressed)
        .onTrue(stepUp())
        // (Driver) Top DPad -> goto max height (Held)
        .whileTrue(topHeight());

    driverController
        .getDPadButton(Direction.DOWN)
        // (Driver) Bottom DPad -> Iterate Backward (Pressed)
        .onTrue(stepDown())
        // (Driver) Bottom DPad -> goto min height (Held)
        .whileTrue(bottomHeight());

    // (Driver) Right DPad -> Set Height
    driverController
        .getDPadButton(Direction.RIGHT)
        .whileTrue(
            new RunEndCommand(
                () -> {
                  elevator.setCurrentPosition();
                },
                () -> {
                  elevator.stopMotors();
                },
                elevator));

    // (Driver) Right Bumper -> Balance on Beam
    driverController.getButton(Button.RightBumper).whileTrue(balanceOnBeam);
  }

  public Command getAutonomousCommand() {

    return autonChooser.getSelected();
  }

  private Command zeroGyro() {
    return new InstantCommand(driveTrain::zeroGyro, driveTrain);
  }

  private Command togglePositionMode() {
    return new InstantCommand(elevator::togglePositionMode, elevator);
  }

  private Command stepUp() {
    return new InstantCommand(elevator::stepUp, elevator);
  }

  private Command stepDown() {
    return new InstantCommand(elevator::stepDown, elevator);
  }

  private Command bottomHeight() {
    return runAferSomeTime(elevator::bottomHeight, 1, elevator);
  }

  private Command topHeight() {
    return runAferSomeTime(elevator::topHeight, 1, elevator);
  }

  private Command runAferSomeTime(Runnable func, double seconds, SubsystemBase subsytem) {
    return new WaitCommand(seconds).andThen(func, subsytem);
  }
}
