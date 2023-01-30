package com.rambots4571.chargedup.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.rambots4571.chargedup.robot.Constants.Cvator;
import com.rambots4571.chargedup.robot.Constants.Settings;
import com.rambots4571.rampage.motor.TalonPID;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

public class Elevator extends SubsystemBase {

  private final WPI_TalonFX baseMotorMaster, baseMotorFollower;
  private final List<WPI_TalonFX> allMotors;

  private final DigitalInput limitSwitch;

  private final TalonPID baseMotorController;

  private static Elevator instance = new Elevator();

  private static Map<Position, Double> heights = new HashMap<>();

  static {
    heights.put(new Position(PositionMode.Cone, Height.Bottom), 0.0);
    heights.put(new Position(PositionMode.Cone, Height.Middle), null);
  }

  public static Elevator getInstance() {
    return instance;
  }

  private Elevator() {
    baseMotorMaster = new WPI_TalonFX(Cvator.BASE_MOTOR_MASTER);
    baseMotorFollower = new WPI_TalonFX(Cvator.BASE_MOTOR_FOLLOWER);

    allMotors = Arrays.asList(baseMotorMaster, baseMotorFollower);

    allMotors.forEach(
        motor -> {
          motor.configFactoryDefault();
          motor.setNeutralMode(Cvator.MODE);
          motor.configOpenloopRamp(Cvator.rampRate, Settings.timeoutMs);
          motor.configStatorCurrentLimit(Cvator.statorLimit);
          motor.configSupplyCurrentLimit(Cvator.supplyLimit);
          motor.enableVoltageCompensation(true);
          motor.configVoltageCompSaturation(12, Settings.timeoutMs);
        });

    baseMotorMaster.setInverted(Cvator.masterInvert);
    baseMotorFollower.setInverted(Cvator.followerInvert);

    baseMotorFollower.follow(baseMotorMaster);

    baseMotorController = new TalonPID(baseMotorMaster);
    baseMotorController.setPIDF(Cvator.kP, Cvator.kI, Cvator.kD, Cvator.kF);

    addChild("BaseMotor PID", baseMotorController.getTuner());

    limitSwitch = new DigitalInput(Cvator.LIMITSWITCH);
  }

  public void configMotionMagic() {
    baseMotorMaster.configSelectedFeedbackSensor(
        FeedbackDevice.IntegratedSensor, 0, Settings.timeoutMs);
    baseMotorMaster.setStatusFramePeriod(
        StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Settings.timeoutMs);
    baseMotorMaster.setStatusFramePeriod(
        StatusFrameEnhanced.Status_10_MotionMagic, 10, Settings.timeoutMs);

    baseMotorMaster.configNominalOutputForward(0, Settings.timeoutMs);
    baseMotorMaster.configNominalOutputReverse(0, Settings.timeoutMs);
    baseMotorMaster.configPeakOutputForward(1, Settings.timeoutMs);
    baseMotorMaster.configPeakOutputReverse(-1, Settings.timeoutMs);

    baseMotorMaster.selectProfileSlot(0, 0);
    baseMotorMaster.configClosedloopRamp(0.15, Settings.timeoutMs);

    baseMotorMaster.configMotionCruiseVelocity(Cvator.cruiseVel, Settings.timeoutMs);
    baseMotorMaster.configMotionAcceleration(Cvator.motionAccel, Settings.timeoutMs);
  }

  public boolean isLimitSwitchPressed() {
    return !limitSwitch.get();
  }

  public void setBaseMotor(double speed) {
    if (isLimitSwitchPressed() && speed < 0) {
      stopMotors();
    } else {
      baseMotorMaster.set(ControlMode.PercentOutput, speed);
    }
  }

  public void stopMotors() {
    baseMotorMaster.set(0);
  }

  public static enum PositionMode {
    Cone,
    Cube
  }

  public static enum Height {
    Bottom,
    Middle,
    Top
  }

  public static class Position {
    private PositionMode mode;
    private Height height;

    public Position(PositionMode mode, Height height) {
      this.mode = mode;
      this.height = height;
    }

    @Override
    public boolean equals(Object o) {
      if (this == o) return true;
      if (!(o instanceof Position)) return false;
      Position position = (Position) o;
      return mode == position.mode && height == position.height;
    }

    @Override
    public int hashCode() {
      return Objects.hash(mode, height);
    }

    @Override
    public String toString() {
      return "Position{" + "mode=" + mode + ", height=" + height + '}';
    }
  }
}
