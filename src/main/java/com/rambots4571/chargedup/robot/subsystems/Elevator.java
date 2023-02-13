package com.rambots4571.chargedup.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.rambots4571.chargedup.robot.Constants.Cvator;
import com.rambots4571.chargedup.robot.Constants.Cvator.ArmPIDF;
import com.rambots4571.chargedup.robot.Constants.Cvator.ElevatorPIDF;
import com.rambots4571.chargedup.robot.Constants.Cvator.Height;
import com.rambots4571.chargedup.robot.Constants.Cvator.PositionMode;
import com.rambots4571.chargedup.robot.Constants.Settings;
import com.rambots4571.rampage.motor.TalonPID;
import com.rambots4571.rampage.util.LinkedList;
import com.rambots4571.rampage.util.LinkedList.Node;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Function;

public class Elevator extends SubsystemBase {

  private final WPI_TalonFX baseMotorMaster, baseMotorFollower, armMotor;
  private final TalonPID baseMotorController, armController;

  private final DigitalInput limitSwitch;

  private final LinkedList<Pair<Height, Height>> heights = new LinkedList<>();
  private Node<Pair<Height, Height>> currNode;

  private PositionMode mode = PositionMode.CONE;

  private Function<Node<Pair<Height, Height>>, Height> getHeight =
      node -> node.getItem().getFirst();

  private static Elevator instance = new Elevator();

  public static Elevator getInstance() {
    return instance;
  }

  private Elevator() {
    baseMotorMaster = new WPI_TalonFX(Cvator.BASE_MOTOR_MASTER);
    baseMotorFollower = new WPI_TalonFX(Cvator.BASE_MOTOR_FOLLOWER);

    armMotor = new WPI_TalonFX(Cvator.ARM_MOTOR);

    List.of(baseMotorMaster, baseMotorFollower, armMotor)
        .forEach(
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

    configMotionMagic();

    baseMotorFollower.follow(baseMotorMaster);

    baseMotorController = new TalonPID(baseMotorMaster);
    baseMotorController.setPIDF(ElevatorPIDF.kP, ElevatorPIDF.kI, ElevatorPIDF.kD, ElevatorPIDF.kF);

    armController = new TalonPID(armMotor);
    armController.setPIDF(ArmPIDF.kP, ArmPIDF.kI, ArmPIDF.kD, ArmPIDF.kF);

    addChild("BaseMotor PID", baseMotorController.getTuner());

    limitSwitch = new DigitalInput(Cvator.LIMITSWITCH);

    addHeights();
    currNode = heights.getFirst();
  }

  private void addHeightPair(Height cone, Height cube) {
    heights.add(new Pair<Height, Height>(cone, cube));
  }

  private void addHeights() {
    addHeightPair(Height.CONE_BOTTOM, Height.CUBE_BOTTOM);
    addHeightPair(Height.CONE_MIDDLE, Height.CUBE_MIDDLE);
    addHeightPair(Height.CONE_TOP, Height.CUBE_TOP);
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

  public void togglePositionMode() {
    if (mode == PositionMode.CONE) {
      getHeight = node -> node.getItem().getSecond();
      mode = PositionMode.CUBE;
    } else {
      getHeight = node -> node.getItem().getFirst();
      mode = PositionMode.CONE;
    }
  }

  public void stepUp() {
    if (currNode.getNext() != null) {
      currNode = currNode.getNext();
    }
  }

  public void stepDown() {
    if (currNode.getPrev() != null) {
      currNode = currNode.getPrev();
    }
  }

  public void bottomHeight() {
    currNode = heights.getFirst();
  }

  public void topHeight() {
    currNode = heights.getLast();
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

  public void setCurrentPosition() {
    setHeight(getHeight.apply(currNode));
  }

  public void setHeight(DoubleSupplier height) {
    baseMotorMaster.set(ControlMode.MotionMagic, height.getAsDouble());
  }

  public double getRawEncoderPosition() {
    return baseMotorController.getRaw();
  }

  public double getRawSpeed() {
    return baseMotorMaster.getSelectedSensorVelocity();
  }

  @Override
  public void periodic() {
    // setCurrentPosition();
  }
}
