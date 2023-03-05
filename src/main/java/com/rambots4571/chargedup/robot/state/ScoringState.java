package com.rambots4571.chargedup.robot.state;

import com.rambots4571.chargedup.robot.Constants.Cvator.PositionMode;
import com.rambots4571.chargedup.robot.subsystems.Arm;
import com.rambots4571.chargedup.robot.subsystems.Claw;

import lombok.Value;

public class ScoringState {
  @Value
  public static class Position {
    private double armLength; // raw units
    private double elbowAngle; // rads or degrees
    private double wristPosition; // raw units
  }

  private final Arm arm;
  private final Claw claw;

  private static final Position[] cubePositions =
      new Position[] {new Position(0, 0, 0), new Position(0, 0, 0), new Position(0, 0, 0)};

  private static final Position[] conePositions =
      new Position[] {new Position(0, 0, 0), new Position(0, 0, 0), new Position(0, 0, 0)};

  private static final Position feederPosition = 
      new Position(0, 0, 0);

  private Position[] currPositions;

  // starting position mode
  private PositionMode mode = PositionMode.CONE;

  private int index = 0;

  public ScoringState(Arm arm, Claw claw) {
    this.arm = arm;
    this.claw = claw;
    currPositions = mode == PositionMode.CONE ? conePositions : cubePositions;
  }

  public Position currPosition() {
    return currPositions[index];
  }

  public void stepUp() {
    if (index == currPositions.length - 1) return;
    index++;
    updatePostion();
  }

  public void stepDown() {
    if (index == 0) return;
    index--;
    updatePostion();
  }

  public void maxPos() {
    index = currPositions.length - 1;
    updatePostion();
  }

  public void minPos() {
    index = 0;
    updatePostion();
  }

  public void feederPos() {
    setState(feederPosition);
  }

  public void setState(Position pos) {
    setState(pos.armLength, pos.elbowAngle);
  }

  public void setState(double length, double angle) {
    arm.setDesiredLength(length);
    // TODO: make sure to convert angle to raw
    arm.setDesiredAngle(angle);
  }

  /** Updates the desired postions in each subsystem which will be set inside their periodic() */
  public void updatePostion() {
    Position pos = currPosition();
    arm.setDesiredLength(pos.armLength);
    // TODO: make sure to convert angle to raw
    arm.setDesiredAngle(pos.elbowAngle);
    claw.setPitch(pos.wristPosition);
  }

  /** Manually go to position (mostly for testing) without the periodic */
  public void goToPosition() {
    Position pos = currPosition();
    arm.setLength(pos.armLength);
    // TODO: make sure to convert angle to raw
    arm.setAngle(pos.elbowAngle);
  }

  public void stop() {
    arm.stopArm();
    arm.stopPivot();
  }

  public void toggleMode() {
    if (mode == PositionMode.CONE) {
      currPositions = cubePositions;
      mode = PositionMode.CUBE;
    } else {
      currPositions = conePositions;
      mode = PositionMode.CONE;
    }
    // prevent index out of bound
    index = index >= currPositions.length ? currPositions.length - 1 : index;
  }
}
