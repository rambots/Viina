package com.rambots4571.chargedup.robot.utils;

import edu.wpi.first.math.util.Units;

import lombok.Builder;
import lombok.Value;

/* Contains values and required settings for common COTS swerve modules. */
@Value
@Builder
public class COTSFalconSwerveConstants {
  private double wheelDiameter;
  private double angleGearRatio;
  private double driveGearRatio;
  private double angleKP;
  private double angleKI;
  private double angleKD;
  private double angleKF;
  private boolean driveMotorInvert;
  private boolean angleMotorInvert;
  private boolean canCoderInvert;

  public double getCircumference() {
    return wheelDiameter * Math.PI;
  }

  /** Swerve Drive Specialties - MK3 Module */
  public static COTSFalconSwerveConstants SDSMK3(double driveGearRatio) {
    return COTSFalconSwerveConstants.builder()
        .wheelDiameter(Units.inchesToMeters(4.0))
        .angleGearRatio(12.8 / 1.0)
        .driveGearRatio(driveGearRatio)
        .angleKP(0.2)
        .angleKI(0)
        .angleKD(0)
        .angleKF(0)
        .driveMotorInvert(false)
        .angleMotorInvert(false)
        .canCoderInvert(false)
        .build();
  }

  /** Swerve Drive Specialties - MK4 Module */
  public static COTSFalconSwerveConstants SDSMK4(double driveGearRatio) {
    return COTSFalconSwerveConstants.builder()
        .wheelDiameter(Units.inchesToMeters(4.0))
        .angleGearRatio(12.8 / 1.0)
        .driveGearRatio(driveGearRatio)
        .angleKP(0)
        .angleKI(0)
        .angleKD(0)
        .angleKF(0)
        .driveMotorInvert(false)
        .angleMotorInvert(false)
        .canCoderInvert(false)
        .build();
  }

  /** Swerve Drive Specialties - MK4i Module */
  public static COTSFalconSwerveConstants SDSMK4i(double driveGearRatio) {
    double wheelDiameter = Units.inchesToMeters(4.0);

    /** (150 / 7) : 1 */
    double angleGearRatio = ((150.0 / 7.0) / 1.0);

    double angleKP = 0.3;
    double angleKI = 0.0;
    double angleKD = 0.0;
    double angleKF = 0.0;

    boolean driveMotorInvert = false;
    boolean angleMotorInvert = true;
    boolean canCoderInvert = false;
    return new COTSFalconSwerveConstants(
        wheelDiameter,
        angleGearRatio,
        driveGearRatio,
        angleKP,
        angleKI,
        angleKD,
        angleKF,
        driveMotorInvert,
        angleMotorInvert,
        canCoderInvert);
  }

  /* Drive Gear Ratios for all supported modules */
  public static class driveGearRatios {
    /* SDS MK3 */
    /** SDS MK3 - 8.16 : 1 */
    public static final double SDSMK3_Standard = (8.16 / 1.0);
    /** SDS MK3 - 6.86 : 1 */
    public static final double SDSMK3_Fast = (6.86 / 1.0);

    /* SDS MK4 */
    /** SDS MK4 - 8.14 : 1 */
    public static final double SDSMK4_L1 = (8.14 / 1.0);
    /** SDS MK4 - 6.75 : 1 */
    public static final double SDSMK4_L2 = (6.75 / 1.0);
    /** SDS MK4 - 6.12 : 1 */
    public static final double SDSMK4_L3 = (6.12 / 1.0);
    /** SDS MK4 - 5.14 : 1 */
    public static final double SDSMK4_L4 = (5.14 / 1.0);

    /* SDS MK4i */
    /** SDS MK4i - 8.14 : 1 */
    public static final double SDSMK4i_L1 = (8.14 / 1.0);
    /** SDS MK4i - 6.75 : 1 */
    public static final double SDSMK4i_L2 = (6.75 / 1.0);
    /** SDS MK4i - 6.12 : 1 */
    public static final double SDSMK4i_L3 = (6.12 / 1.0);
  }
}
