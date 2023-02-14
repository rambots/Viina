// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.rambots4571.chargedup.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.rambots4571.chargedup.robot.Constants.DriveConstants;
import com.rambots4571.chargedup.robot.Constants.Settings;
import com.rambots4571.chargedup.robot.utils.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {

  private final SwerveModule[] modules;
  private final SwerveDriveOdometry m_Odometry;

  private final Pigeon2 m_gyro;

  private static DriveTrain instance = new DriveTrain();

  public static DriveTrain getInstance() {
    return instance;
  }

  public DriveTrain() {
    m_gyro = new Pigeon2(DriveConstants.PIGEON_IMU_2);
    m_gyro.configFactoryDefault();
    zeroGyro();

    modules =
        new SwerveModule[] {
          new SwerveModule(0, DriveConstants.Mod0.constants),
          new SwerveModule(1, DriveConstants.Mod1.constants),
          new SwerveModule(2, DriveConstants.Mod2.constants),
          new SwerveModule(3, DriveConstants.Mod3.constants)
        };

    Timer.delay(1.0);
    resetModulesToAbsolute();

    m_Odometry =
        new SwerveDriveOdometry(
            DriveConstants.kDriveKinematics, getRotation2d(), getModulePositions());

    resetOdometry(Settings.STARTING_POSITION);

    for (SwerveModule mod : modules) {
      DriverStation.reportError(
          "CANcoder on Module "
              + mod.moduleNumber
              + " took "
              + mod.CANcoderInitTime
              + " ms to be ready.",
          false);
    }
  }

  // *****************************************
  // ************** Driving ******************
  // *****************************************

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] states =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getRotation2d())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));

    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxSpeedMetersPerSecond);

    for (SwerveModule mod : modules) {
      mod.setDesiredState(states[mod.moduleNumber], isOpenLoop);
    }
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);

    for (SwerveModule mod : modules) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];

    for (SwerveModule mod : modules) {
      states[mod.moduleNumber] = mod.getPosition();
    }
    return states;
  }

  public void resetModulesToAbsolute() {
    for (SwerveModule mod : modules) {
      mod.resetToAbsolute();
    }
  }

  public void stopMotors() {
    for (SwerveModule mod : modules) {
      mod.stopMotors();
    }
  }

  // *****************************************
  // ************* Robot Angle ***************
  // *****************************************

  public double getGyroAngle() {
    return m_gyro.getYaw() % 360;
  }

  public double getGyroPitch() {
    return m_gyro.getPitch();
  }

  public Rotation2d getRotation2d() {
    return (DriveConstants.invertGyro)
        ? Rotation2d.fromDegrees(360 - m_gyro.getYaw())
        : Rotation2d.fromDegrees(m_gyro.getYaw());
  }

  public void zeroGyro() {
    m_gyro.setYaw(0);
  }

  // *****************************************
  // ************* Robot Angle ***************
  // *****************************************

  public void updateOdometry() {
    m_Odometry.update(getRotation2d(), getModulePositions());
  }

  public void resetOdometry(Pose2d pose) {
    m_Odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  public Pose2d getPose() {
    return m_Odometry.getPoseMeters();
  }

  @Override
  public void periodic() {
    updateOdometry();

    for (SwerveModule mod : modules) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }
  }

  @Override
  public void simulationPeriodic() {}
}
