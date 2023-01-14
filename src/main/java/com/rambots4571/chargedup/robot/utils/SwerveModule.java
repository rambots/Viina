package com.rambots4571.chargedup.robot.utils;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.rambots4571.chargedup.robot.Constants.DriveConstants;
import com.rambots4571.rampage.math.Converter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    
    public int moduleNumber;
    private final Rotation2d angleOffset;
    private final Rotation2d lastAngle;

    private final TalonFX driveMotor;
    private final TalonFX turnMotor;
    private final CANCoder angleEncoder;

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
    
        driveMotor = new TalonFX(moduleConstants.driveMotorID);

        turnMotor = new TalonFX(moduleConstants.angleMotorID);

        angleEncoder = new CANCoder(moduleConstants.cancoderID);

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond;
            driveMotor.set(ControlMode.PercentOutput, percentOutput);
        } else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, DriveConstants.kWheelCircumference, DriveConstants.kDriveGearRatio);
            driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, DriveConstants.swerveFF.calculate(desiredState.speedMetersPerSecond));
        }
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(turnMotor.getSelectedSensorPosition(), DriveConstants.kTurnGrearRatio));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            Conversions.falconToMPS(driveMotor.getSelectedSensorVelocity(), 
            DriveConstants.kWheelCircumference, DriveConstants.kDriveGearRatio), getAngle());
    }
}
