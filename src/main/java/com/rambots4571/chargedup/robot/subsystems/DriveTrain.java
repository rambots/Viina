// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.rambots4571.chargedup.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import com.rambots4571.chargedup.robot.Constants.DriveConstants;

import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {

  private final SwerveModule frontLeft, frontRight, backLeft, backRight;

  private final ShuffleboardTab swerveTab;

  private final SwerveDriveOdometry m_Odometry;
  private final Field2d field;

  private final Pigeon2 m_gyro;

  private static DriveTrain instance = new DriveTrain();

  public static DriveTrain getInstance() {
    return instance;
  }

  public DriveTrain() {
    swerveTab = Shuffleboard.getTab("DriveTrain");

    frontLeft =
        Mk4iSwerveModuleHelper.createFalcon500(
            swerveTab
                .getLayout("Front Left Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(0, 0),
            Mk4iSwerveModuleHelper.GearRatio.L2,
            DriveConstants.FRONT_LEFT_DRIVE_MOTOR,
            DriveConstants.FRONT_LEFT_STEER_MOTOR,
            DriveConstants.FRONT_LEFT_STEER_ENCODER,
            DriveConstants.FRONT_LEFT_STEER_OFFSET);

    frontRight =
        Mk4iSwerveModuleHelper.createFalcon500(
            swerveTab
                .getLayout("Front Right Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(2, 0),
            Mk4iSwerveModuleHelper.GearRatio.L2,
            DriveConstants.FRONT_RIGHT_DRIVE_MOTOR,
            DriveConstants.FRONT_RIGHT_STEER_MOTOR,
            DriveConstants.FRONT_RIGHT_STEER_ENCODER,
            DriveConstants.FRONT_RIGHT_STEER_OFFSET);

    backLeft =
        Mk4iSwerveModuleHelper.createFalcon500(
            swerveTab
                .getLayout("Back Left Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(4, 0),
            Mk4iSwerveModuleHelper.GearRatio.L2,
            DriveConstants.BACK_LEFT_DRIVE_MOTOR,
            DriveConstants.BACK_LEFT_STEER_MOTOR,
            DriveConstants.BACK_LEFT_STEER_ENCODER,
            DriveConstants.BACK_LEFT_STEER_OFFSET);

    backRight =
        Mk4iSwerveModuleHelper.createFalcon500(
            swerveTab
                .getLayout("Back Right Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(6, 0),
            Mk4iSwerveModuleHelper.GearRatio.L2,
            DriveConstants.BACK_RIGHT_DRIVE_MOTOR,
            DriveConstants.BACK_RIGHT_STEER_MOTOR,
            DriveConstants.BACK_RIGHT_STEER_ENCODER,
            DriveConstants.BACK_RIGHT_STEER_OFFSET);

    m_gyro = new Pigeon2(DriveConstants.PIGEON_IMU_2);
  }

  // *****************************************
  // ************** Driving ******************
  // *****************************************

  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxSpeedMetersPerSecond);
    frontLeft.set(states[0].speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond * DriveConstants.kMaxVoltage, states[0].angle.getRadians());
    frontRight.set(states[1].speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond * DriveConstants.kMaxVoltage, states[1].angle.getRadians());
    backLeft.set(states[2].speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond * DriveConstants.kMaxVoltage, states[2].angle.getRadians());
    backRight.set(states[3].speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond * DriveConstants.kMaxVoltage, states[3].angle.getRadians());

  }

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
