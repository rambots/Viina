package com.rambots4571.chargedup.robot.state;

import com.rambots4571.chargedup.robot.Constants.Cvator.Position;
import com.rambots4571.chargedup.robot.Constants.Cvator.PositionMode;
import com.rambots4571.chargedup.robot.subsystems.Arm;
import com.rambots4571.chargedup.robot.subsystems.Elevator;

import java.util.function.Function;

public class ScoringState {
  private final Position[] positions =
      new Position[] {
        Position.BOTTOM, Position.MIDDLE, Position.TOP,
      };

  private int index = 0;

  private PositionMode mode;

  private Function<Position, Double> getHeight = pos -> pos.getConeHeight();

  private final Elevator elevator;

  private final Arm arm;

  public ScoringState(Elevator elevator, Arm arm) {
    this.elevator = elevator;
    this.arm = arm;
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

  public void updatePostion() {
    Position pos = currPosition();
    elevator.setDesiredHeight(getHeight.apply(pos));
    arm.setDesiredLength(pos.getArmlength());
  }

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
