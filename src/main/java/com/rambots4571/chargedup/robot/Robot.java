// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.rambots4571.chargedup.robot;

import com.rambots4571.chargedup.robot.subsystems.DriveTrain;
import com.rambots4571.chargedup.robot.utils.CTREConfigs;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  public static CTREConfigs ctreConfigs;

  private Command autoCommand;

  private RobotContainer container;

  public static DriveTrain driveTrain;

  @Override
  public void robotInit() {
    ctreConfigs = new CTREConfigs();

    container = new RobotContainer();

    driveTrain = new DriveTrain();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    autoCommand = container.getAutonomousCommand();

    if (autoCommand != null) {
      autoCommand.schedule();
    }

    driveTrain.resetModulesToAbsolute();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (autoCommand != null) {
      autoCommand.cancel();
    }

    driveTrain.resetModulesToAbsolute();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}
}
