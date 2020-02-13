/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import static frc.robot.Constants.turretConstants.kSoftMaxTurretAngle;
import static frc.robot.Constants.turretConstants.kSoftMinTurretAngle;
import static frc.robot.Constants.turretConstants.kDegreesPerTick;
import static frc.robot.Constants.turretConstants.kTimeout;
import static frc.robot.Constants.turretConstants.kIndex;

public class turretSubsystem extends SubsystemBase {

  private TalonSRX turretDrive = new TalonSRX(Constants.turretConstants.turret);
  private DigitalInput limit1 = new DigitalInput(7);
  private DigitalInput limit2 = new DigitalInput(8);

  public turretSubsystem() {
    turretDrive.configFactoryDefault();
    turretDrive.configSelectedFeedbackSensor(FeedbackDevice.Analog, kIndex, kTimeout);

    // fix rotational direction
    turretDrive.setInverted(false);
    turretDrive.setSensorPhase(false);

    // set soft limits
    turretDrive.configForwardSoftLimitThreshold((int) (kSoftMaxTurretAngle / kDegreesPerTick));
    turretDrive.configReverseSoftLimitThreshold((int) (kSoftMinTurretAngle / kDegreesPerTick));
    turretDrive.configForwardSoftLimitEnable(true);
    turretDrive.configReverseSoftLimitEnable(true);

    // TODO: set max speed, max voltage, max current
    turretDrive.configContinuousCurrentLimit(25);

    // zero the position. start position becomes center
    turretDrive.setSelectedSensorPosition(0, kIndex, kTimeout);

    // TODO: tune PIDF parameters (these are only a guess)
    turretDrive.configAllowableClosedloopError(0, kIndex, kTimeout);
    turretDrive.config_kF(kIndex, 0.01, kTimeout);
    turretDrive.config_kP(kIndex, 0.1, kTimeout);
    turretDrive.config_kI(kIndex, 0, kTimeout);
    turretDrive.config_kD(kIndex, 0, kTimeout);

    // TODO: set Motion Magic max Cruise Velocity and max acceleration

  }

  public void turretHome() {
    turretDrive.set(ControlMode.Position, 0);
  }

  /**
   * setAngleDegrees - turn turret to a given angle relative to robot
   * 
   * @param agnleDeg angle in degrees
   */
  public void setAngleDegrees(double angleDeg) {
    if (angleDeg < kSoftMinTurretAngle) {
      angleDeg = kSoftMinTurretAngle;
    }
    if (angleDeg > kSoftMaxTurretAngle) {
      angleDeg = kSoftMaxTurretAngle;
    }

    turretDrive.set(ControlMode.Position, angleDeg / kDegreesPerTick);
  }

  /**
   * setAngleRadians - turn turret to a given angle relative to robot
   * 
   * @param agnleRad angle in radians
   */
  public void setAngleRadians(double angleRad) {
    setAngleDegrees(angleRad * 180.0 / Math.PI);
  }

  /**
   *  stop - stop the turret motor, disabling PID position control
   */
  public void stop() {
    setPercentOutput(0);
  }

  public void setPercentOutput(double percent) {
    turretDrive.set(ControlMode.PercentOutput, percent);
  }

  public double getAngleDegrees() {
    return(turretDrive.getSelectedSensorPosition() * kDegreesPerTick);
  }

  public double getAngleRadians() {
    return(getAngleDegrees() * Math.PI / 180.0);
  }

  @Override
  public void periodic() {
    boolean turretLimit1 = limit1.get();
    boolean turretLimit2 = limit2.get();
    int pos = turretDrive.getSelectedSensorPosition();

    SmartDashboard.putBoolean("TurretLimit 1", turretLimit1);
    SmartDashboard.putBoolean("TurretLimit 2", turretLimit2);
    SmartDashboard.putNumber("Turret Pos", pos);
    SmartDashboard.putNumber("Turret Angle", pos * kDegreesPerTick);

    if (turretLimit1 == true) {
      turretDrive.set(ControlMode.PercentOutput, 0.0);
      DriverStation.reportError("Min limit Reached on turret. motor stopped", false);
      // check angle and reset position to kSoftMinTurretAngle if off by more than 1 deg
      if (Math.abs(pos * kDegreesPerTick - kSoftMinTurretAngle) > 1.0) {
        // TODO: magnetic limits switch may be outside software min/max set accordingly
        turretDrive.setSelectedSensorPosition((int) (kSoftMinTurretAngle / kDegreesPerTick), kIndex, kTimeout);
      }
    }

    if (turretLimit2 == true) {
      turretDrive.set(ControlMode.PercentOutput, 0.0);
      DriverStation.reportError("Max limit Reached on turret, motor stopped", false);
      // check angle and reset position to kSoftMaxTurretAngle if off by more than 1 deg
      if (Math.abs(pos * kDegreesPerTick - kSoftMaxTurretAngle) > 1.0) {
        // TODO: magnetic limits switch may be outside software min/max set accordingly
        turretDrive.setSelectedSensorPosition((int) (kSoftMaxTurretAngle / kDegreesPerTick), kIndex, kTimeout);
      }
    }
  }
}
