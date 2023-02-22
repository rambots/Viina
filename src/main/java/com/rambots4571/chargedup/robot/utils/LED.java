package com.rambots4571.chargedup.robot.utils;

import com.rambots4571.rampage.led.Pattern;
import com.rambots4571.rampage.led.REVBlinkin;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  private final REVBlinkin blinkin;
  private Pattern pattern = Pattern.Twinkle.OCEAN;

  private static LED led = new LED();

  private LED() {
    // TODO update channel
    this.blinkin = new REVBlinkin(99);
  }

  public static LED getInstance() {
    return led;
  }

  public void setPattern(Pattern pattern) {
    this.pattern = pattern;
  }

  @Override
  public void periodic() {
    blinkin.setPattern(pattern);
  }
}
