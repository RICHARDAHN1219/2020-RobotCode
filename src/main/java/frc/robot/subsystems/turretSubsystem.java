/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
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
import frc.robot.Constants.turretConstants;
import static frc.robot.Constants.turretConstants.kSoftMaxTurretAngle;
import static frc.robot.Constants.turretConstants.kSoftMinTurretAngle;

public class turretSubsystem extends SubsystemBase {

  private TalonSRX turretDrive = new TalonSRX(Constants.turretConstants.turret);
  private DigitalInput limit1 = new DigitalInput(7);
  private DigitalInput limit2 = new DigitalInput(8);
  
  public turretSubsystem() {
    turretDrive.configFactoryDefault();
    turretDrive.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 10);
    
    // fix rotational direction
    turretDrive.setInverted(false);
    turretDrive.setSensorPhase(false);

    // set soft limits
    turretDrive.configForwardSoftLimitThreshold((int) (kSoftMaxTurretAngle / (360.0 * Constants.turretConstants.kTurretRotationsPerTick)));
    turretDrive.configReverseSoftLimitThreshold((int) (kSoftMinTurretAngle / (360.0 * Constants.turretConstants.kTurretRotationsPerTick)));
    turretDrive.configForwardSoftLimitEnable(true);
    turretDrive.configReverseSoftLimitEnable(true);

    turretDrive.configContinuousCurrentLimit(25);
    
    // zero the position. start position becomes center
    turretDrive.setSelectedSensorPosition(0, 0, 10); 
    turretDrive.getSensorCollection().setQuadraturePosition(0, 10);

    // TODO: make sure position is zerod correctly
    // TODO: set max speed, max voltage, max current
    // TODO: set PID parameters
  }
  
  public void turretHome() {
    turretDrive.set(ControlMode.Position, 0);
  }

  public void setAngleDegrees(double angleDeg) {
    if (angleDeg < kSoftMinTurretAngle) {
      angleDeg = kSoftMinTurretAngle;
    }
    if (angleDeg > kSoftMaxTurretAngle) {
      angleDeg = kSoftMaxTurretAngle;
    }

    turretDrive.set(ControlMode.Position, angleDeg / kDegreesPerTick);
  }

  public void setAngleRadians(double angleRad) {
    setAngleDegrees(angleRad * 180.0 / Math.PI );
  }

  public void setPercentOutput(double percent) {
    turretDrive.set(ControlMode.PercentOutput, percent);
  }

  @Override
  public void periodic() {
    boolean turretLimit1 = limit1.get();
    boolean turretLimit2 = limit2.get();

    SmartDashboard.putBoolean("TurretLimit 1", turretLimit1);
    SmartDashboard.putBoolean("TurretLimit 2", turretLimit2);
    SmartDashboard.putNumber("Turret Pos", turretDrive.getSelectedSensorPosition());
    SmartDashboard.putNumber("Turret Angle", turretDrive.getSelectedSensorPosition() * 
                                             kDegreesPerTick);

    if (turretLimit1 == true) {
      turretDrive.set(ControlMode.PercentOutput, 0.0);
      DriverStation.reportError("Min limit Reached on turret. motor stopped", false);
      // TODO: reset position to kSoftMinTurretAngle
    }
    
    if (turretLimit2 == true) {
      turretDrive.set(ControlMode.PercentOutput, 0.0);
      DriverStation.reportError("Max limit Reached on turret, motor stopped", false);
      // TODO: reset position to kSoftMaxTurretAngle
    }    
  }
}
