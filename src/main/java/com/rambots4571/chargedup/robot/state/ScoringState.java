package com.rambots4571.chargedup.robot.state;

import com.rambots4571.chargedup.robot.Constants.Cvator.Position;
import com.rambots4571.chargedup.robot.Constants.Cvator.PositionMode;
import com.rambots4571.chargedup.robot.subsystems.Arm;
import com.rambots4571.chargedup.robot.subsystems.Elevator;

import java.util.function.Function;

public class ScoringState {
  private final Elevator elevator;
  private final Arm arm;

  // starting position mode
  private PositionMode mode = PositionMode.CONE;

  private final Position[] positions =
      new Position[] {
        Position.BOTTOM, Position.MIDDLE, Position.TOP,
      };

  private int index = 0;

  private Function<Position, Double> getHeight;

  public ScoringState(Elevator elevator, Arm arm) {
    this.elevator = elevator;
    this.arm = arm;

    getHeight = mode == PositionMode.CONE ? pos -> pos.getConeHeight() : pos -> pos.getCubeHeight();
  }

  public Position currPosition() {
    return positions[index];
  }

  public void stepUp() {
    if (index == positions.length - 1) return;
    index++;
    updatePostion();
  }

  public void stepDown() {
    if (index == 0) return;
    index--;
    updatePostion();
  }

  public void maxPos() {
    index = positions.length - 1;
    updatePostion();
  }

  public void minPos() {
    index = 0;
    updatePostion();
  }

  public void setState(Position pos, PositionMode mode) {
    double height = mode == PositionMode.CONE ? pos.getConeHeight() : pos.getCubeHeight();
    setState(height, pos.getArmlength());
  }

  public void setState(double height, double length) {
    elevator.setDesiredHeight(length);
    arm.setDesiredLength(length);
  }

  /** Updates the desired postions in each subsystem which will be set inside their periodic() */
  public void updatePostion() {
    Position pos = currPosition();
    elevator.setDesiredHeight(getHeight.apply(pos));
    arm.setDesiredLength(pos.getArmlength());
  }

  /** Manually go to position (mostly for testing) without the periodic */
  public void goToPosition() {
    Position pos = currPosition();
    elevator.setHeight(getHeight.apply(pos));
    arm.setLength(pos.getArmlength());
  }

  public void stop() {
    elevator.stopMotors();
    arm.stop();
  }

  public void toggleMode() {
    if (mode == PositionMode.CONE) {
      getHeight = pos -> pos.getCubeHeight();
      mode = PositionMode.CUBE;
    } else {
      getHeight = pos -> pos.getConeHeight();
      mode = PositionMode.CONE;
    }
  }
}