package com.rambots4571.chargedup.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import com.rambots4571.chargedup.robot.SwerveModule;
import com.rambots4571.chargedup.robot.Constants.DriveConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  
  public SwerveDriveOdometry swerveOdometry;
  public SwerveModule[] mSwerveMods;
  public Pigeon2 gyro;

  private static DriveTrain instance = new DriveTrain();

  public static DriveTrain getInstance() {
    return instance;
  }

  public DriveTrain() {
    gyro = new Pigeon2(DriveConstants.pigeonID, "BOYSALIAR");
    gyro.configFactoryDefault();
    zeroGyro();

    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, DriveConstants.Mod0.constants),
          new SwerveModule(1, DriveConstants.Mod1.constants),
          new SwerveModule(2, DriveConstants.Mod2.constants),
          new SwerveModule(3, DriveConstants.Mod3.constants)
        };

    Timer.delay(1.0);
    resetModulesToAbsolute();

    swerveOdometry =
        new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getRotation2d(), getModulePositions());
  }

  // *****************************************
  // ************** Driving ******************
  // *****************************************

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getRotation2d())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];

    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getPosition();
    }
    return states;
  }

  public void resetModulesToAbsolute() {
    for (SwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
    }
  }

  public void stopMotors() {
    for (SwerveModule mod : mSwerveMods) {
      mod.stopMotors();
    }
  }

  // *****************************************
  // ************* Robot Angle ***************
  // *****************************************

  public double getGyroAngle() {
    return gyro.getYaw() % 360;
  }

  public double getGyroPitch() {
    return gyro.getPitch();
  }

  public Rotation2d getRotation2d() {
    return (DriveConstants.invertGyro)
        ? Rotation2d.fromDegrees(360 - gyro.getYaw())
        : Rotation2d.fromDegrees(gyro.getYaw());
  }

  public void zeroGyro() {
    gyro.setYaw(0);
  }

  // *****************************************
  // ************* Robot Angle ***************
  // *****************************************

  public void updateOdometry() {
    swerveOdometry.update(getRotation2d(), getModulePositions());
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  @Override
  public void periodic() {
    swerveOdometry.update(getRotation2d(), getModulePositions());

    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }
  }
}
