// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.rambots4571.chargedup.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import com.rambots4571.chargedup.robot.Constants.DriveConstants;

import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {

  private final SwerveModule frontLeft, frontRight, backLeft, backRight;

  private final Pigeon2 gyro;

  private static DriveTrain instance = new DriveTrain();

  public static DriveTrain getInstance() {
    return instance;
  }

  public DriveTrain() {
    frontLeft =
        Mk4iSwerveModuleHelper.createFalcon500(
            Mk4iSwerveModuleHelper.GearRatio.L2,
            DriveConstants.FRONT_LEFT_DRIVE_MOTOR,
            DriveConstants.FRONT_LEFT_STEER_MOTOR,
            DriveConstants.FRONT_LEFT_STEER_ENCODER,
            DriveConstants.FRONT_LEFT_STEER_OFFSET);

    frontRight =
        Mk4iSwerveModuleHelper.createFalcon500(
            Mk4iSwerveModuleHelper.GearRatio.L2,
            DriveConstants.FRONT_RIGHT_DRIVE_MOTOR,
            DriveConstants.FRONT_RIGHT_STEER_MOTOR,
            DriveConstants.FRONT_RIGHT_STEER_ENCODER,
            DriveConstants.FRONT_RIGHT_STEER_OFFSET);

    backLeft =
        Mk4iSwerveModuleHelper.createFalcon500(
            Mk4iSwerveModuleHelper.GearRatio.L2,
            DriveConstants.BACK_LEFT_DRIVE_MOTOR,
            DriveConstants.BACK_LEFT_STEER_MOTOR,
            DriveConstants.BACK_LEFT_STEER_ENCODER,
            DriveConstants.BACK_LEFT_STEER_OFFSET);

    backRight =
        Mk4iSwerveModuleHelper.createFalcon500(
            Mk4iSwerveModuleHelper.GearRatio.L2,
            DriveConstants.BACK_RIGHT_DRIVE_MOTOR,
            DriveConstants.BACK_RIGHT_STEER_MOTOR,
            DriveConstants.BACK_RIGHT_STEER_ENCODER,
            DriveConstants.BACK_RIGHT_STEER_OFFSET);

    gyro = new Pigeon2(DriveConstants.PIGEON_IMU_2);
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
