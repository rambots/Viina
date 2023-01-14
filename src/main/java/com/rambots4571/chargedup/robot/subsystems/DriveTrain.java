// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.rambots4571.chargedup.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import com.rambots4571.chargedup.robot.Constants.DriveConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {

  // private final SwerveModule frontLeft, frontRight, backLeft, backRight;

  private final ShuffleboardTab swerveTab;

  // private final SwerveDriveOdometry m_Odometry;
  // private final Field2d field;

  private final Pigeon2 m_gyro;

  private static DriveTrain instance = new DriveTrain();

  public static DriveTrain getInstance() {
    return instance;
  }

  private DriveTrain() {
    swerveTab = Shuffleboard.getTab("DriveTrain");

    m_gyro = new Pigeon2(DriveConstants.PIGEON_IMU_2);
  }

  // *****************************************
  // ************** Driving ******************
  // *****************************************

  // *****************************************
  // ************* Robot Angle ***************
  // *****************************************

  public double getGyroAngle() {
    return m_gyro.getYaw() % 360;
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(m_gyro.getYaw());
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
