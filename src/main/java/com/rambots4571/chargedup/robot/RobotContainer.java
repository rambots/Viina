package com.rambots4571.chargedup.robot;

import com.rambots4571.rampage.command.RunEndCommand;
import com.rambots4571.rampage.controller.Gamepad;
import com.rambots4571.rampage.controller.PS4Controller;
import com.rambots4571.rampage.controller.PS4Controller.Button;
import com.rambots4571.rampage.controller.component.DPadButton.Direction;

import com.pathplanner.lib.auto.SwerveAutoBuilder;

import com.rambots4571.chargedup.robot.Constants.DriveConstants;
import com.rambots4571.chargedup.robot.Constants.Settings;
import com.rambots4571.chargedup.robot.commands.arm.TestArmCommand;
import com.rambots4571.chargedup.robot.commands.arm.TestClaw;
import com.rambots4571.chargedup.robot.commands.drive.BalanceOnBeam;
import com.rambots4571.chargedup.robot.commands.drive.SwerveDriveCommand;
import com.rambots4571.chargedup.robot.state.ScoringState;
import com.rambots4571.chargedup.robot.subsystems.Arm;
import com.rambots4571.chargedup.robot.subsystems.Claw;
import com.rambots4571.chargedup.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  // Joysticks
  public final PS4Controller driverController = new PS4Controller(Settings.DRIVERCONTROLLER);
  public final Gamepad gamepad = new Gamepad(Settings.GAMEPAD);

  public final Trigger robotCentricToggle = driverController.getButton(Button.Circle);

  // Misceallaneous
  public final SwerveAutoBuilder autoBuilder;

  public final SendableChooser<Command> autonChooser = new SendableChooser<>();

  private final ScoringState scoringState;

  // Subsytems
  private final DriveTrain driveTrain;
  private final Arm arm;
  private final Claw claw;

  // Commands
  private final SwerveDriveCommand swerveDriveCommand;
  private final TestArmCommand testArmCommand;
  private final BalanceOnBeam balanceOnBeam;

  public RobotContainer() {
    driveTrain = DriveTrain.getInstance();
    arm = Arm.getInstance();
    claw = Claw.getInstance();

    scoringState = new ScoringState(arm);

    swerveDriveCommand =
        new SwerveDriveCommand(
            driveTrain,
            () -> -driverController.getAxisValue(PS4Controller.Axis.LeftY),
            () -> -driverController.getAxisValue(PS4Controller.Axis.LeftX),
            () -> -driverController.getAxisValue(PS4Controller.Axis.RightX),
            () -> robotCentricToggle.getAsBoolean());

    driveTrain.setDefaultCommand(swerveDriveCommand);

    balanceOnBeam = new BalanceOnBeam(driveTrain);

    testArmCommand = new TestArmCommand(arm, () -> gamepad.getAxisValue(Gamepad.Axis.RightXAxis));

    // TODO: uncomment to test arm
    arm.setDefaultCommand(testArmCommand);

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
    setEventMap();

    configureButtonBindings();
  }

  private void setEventMap() {}

  private void configureButtonBindings() {
    // (Driver) Y -> Reset Gyro
    driverController.getButton(Button.Triangle).onTrue(zeroGyro());

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
            new RunEndCommand(scoringState::goToPosition, scoringState::stop, arm));

    gamepad
        .getButton(Gamepad.Button.X)
        .whileTrue(
            new RunEndCommand(
                () -> {
                  claw.setSpeed(1);
                },
                () -> {
                  claw.stop();
                },
                claw));

    gamepad.getButton(Gamepad.Button.Y).whileTrue(new TestClaw(claw));

    // (Driver) Right Bumper -> Balance on Beam
    driverController.getButton(PS4Controller.Button.R1).whileTrue(balanceOnBeam);
  }

  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }

  private Command zeroGyro() {
    return new InstantCommand(driveTrain::zeroGyro, driveTrain);
  }

  private Command togglePositionMode() {
    return new InstantCommand(scoringState::toggleMode, arm);
  }

  private Command stepUp() {
    return new InstantCommand(scoringState::stepUp, arm);
  }

  private Command stepDown() {
    return new InstantCommand(scoringState::stepDown, arm);
  }

  private Command bottomHeight() {
    return runAfterSomeTime(scoringState::minPos, 1, arm);
  }

  private Command topHeight() {
    return runAfterSomeTime(scoringState::maxPos, 1, arm);
  }

  private Command runAfterSomeTime(Runnable func, double seconds, SubsystemBase... subsytem) {
    return new WaitCommand(seconds).andThen(func, subsytem);
  }
}
