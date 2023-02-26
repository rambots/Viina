package com.rambots4571.chargedup.robot.subsystems;

import com.rambots4571.rampage.motor.TalonPID;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.rambots4571.chargedup.robot.Constants.CTurret;
import com.rambots4571.chargedup.robot.Constants.Settings;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {

  private final WPI_TalonFX turretMotor;
  private final TalonPID turretPID;

  private final DigitalInput leftSwitch, rightSwitch;

  private static Turret instance = new Turret();

  public static Turret getInstance() {
    return instance;
  }

  private Turret() {
    turretMotor = new WPI_TalonFX(CTurret.TURRET_MOTOR, "BOYSALIAR");

    turretPID = new TalonPID(turretMotor);
    turretPID.setPIDF(CTurret.kP, CTurret.kI, CTurret.kD, CTurret.kF);

    addChild("Turret PID", turretPID.getTuner());

    leftSwitch = new DigitalInput(CTurret.LEFT_LIMIT_SWITCH);
    rightSwitch = new DigitalInput(CTurret.RIGHT_LIMIT_SWITCH);

    configMotor();
  }

  private void configMotor() {
    turretMotor.configFactoryDefault();

    turretMotor.setInverted(CTurret.INVERT);
    turretMotor.setNeutralMode(CTurret.MODE);

    turretMotor.configSupplyCurrentLimit(CTurret.SUPPLY_LIMIT);
    turretMotor.configStatorCurrentLimit(CTurret.STATOR_LIMIT);

    turretMotor.configOpenloopRamp(CTurret.RAMP_RATE, Settings.timeoutMs);

    turretMotor.enableVoltageCompensation(true);
    turretMotor.configVoltageCompSaturation(12, Settings.timeoutMs);
  }

  public boolean isLeftSwitchPressed() {
    return !leftSwitch.get();
  }

  public boolean isRightSwitchPressed() {
    return !rightSwitch.get();
  }

  public void stop() {
    turretMotor.set(0);
  }

  public void setTurretMotor(double speed) {
    if (isLeftSwitchPressed() && speed < 0) {
      stop();
    } else if (isRightSwitchPressed() && speed > 0) {
        stop();
    } else {
        turretMotor.set(ControlMode.PercentOutput, speed * CTurret.TURRET_SPEED);
    }
  }
}
