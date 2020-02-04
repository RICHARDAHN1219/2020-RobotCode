/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class turretSubsystem extends SubsystemBase {

  private TalonSRX turretDrive = new TalonSRX(Constants.turretConstants.turret);
  private DigitalInput limit1 = new DigitalInput(7);
  private DigitalInput limit2 = new DigitalInput(8);
  private boolean turretLimit1;
  private boolean turretLimit2;
  
  public turretSubsystem() {
    turretDrive.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 10);
    turretDrive.configForwardSoftLimitEnable(false);
    turretDrive.configReverseSoftLimitEnable(false);
    turretDrive.configForwardSoftLimitThreshold((int) (Constants.turretConstants.kSoftMaxTurretAngle / (360.0 * Constants.turretConstants.kTurretRotationsPerTick)));
    turretDrive.configReverseSoftLimitThreshold((int) (Constants.turretConstants.kSoftMinTurretAngle / (360.0 * Constants.turretConstants.kTurretRotationsPerTick)));
    turretDrive.configContinuousCurrentLimit(25);
    turretDrive.setSelectedSensorPosition(0, 0, 10); 
    turretDrive.getSensorCollection().setQuadraturePosition(0, 10);
    turretDrive.setNeutralMode(NeutralMode.Brake);
  }
  
  public void turretHome() {
    turretDrive.set(ControlMode.Position, 0);
  }

  public void setPercentOutput(double percent) {
    turretDrive.set(ControlMode.PercentOutput, percent);
  }

  @Override
  public void periodic() {
    boolean turretLimit1 = limit1.get();
    boolean turretLimit2 = limit2.get();
    if (turretLimit1 == true) {
      turretDrive.set(ControlMode.PercentOutput, -.5);
      DriverStation.reportError("Limit Reached on turret, going back to safe position.", false);
    }
    
    if (turretLimit2 == true) {
      turretDrive.set(ControlMode.PercentOutput, .5);
      DriverStation.reportError("Limit Reached on turret, going back to safe position.", false);
    }
    
  }
}
