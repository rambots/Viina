// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.rambots4571.chargedup.robot;

import com.rambots4571.chargedup.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

  // Subsytems
  private final DriveTrain driveTrain;

  public RobotContainer() {
    driveTrain = DriveTrain.getInstance();

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {

    return null;
  }
}
