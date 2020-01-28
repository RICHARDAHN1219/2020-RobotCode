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
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class turretSubsystem extends SubsystemBase {

  public static final TalonSRX turretDrive = new TalonSRX(Constants.turretConstants.turret);
  public static DigitalInput limit1 = new DigitalInput(7);
  public static DigitalInput limit2 = new DigitalInput(8);
  public static DigitalInput limit3 = new DigitalInput(9);
  public static boolean turretLimit1;
  public static boolean turretLimit2;
  
  public turretSubsystem() {
    turretDrive.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 10);
    turretDrive.configForwardSoftLimitEnable(true);
    turretDrive.configReverseSoftLimitEnable(true);
    turretDrive.configForwardSoftLimitThreshold((int) (Constants.turretConstants.kSoftMaxTurretAngle / (360.0 * Constants.turretConstants.kTurretRotationsPerTick)));
    turretDrive.configReverseSoftLimitThreshold((int) (Constants.turretConstants.kSoftMinTurretAngle / (360.0 * Constants.turretConstants.kTurretRotationsPerTick)));
    turretSubsystem.turretDrive.configContinuousCurrentLimit(25);
    
  }
  
  public static void turretHome(){
      turretDrive.set(ControlMode.Position, 0);
    }

  @Override
  public void periodic() {
    boolean turretLimit1 = turretSubsystem.limit1.get();
    boolean turretLimit2 = turretSubsystem.limit2.get();
    if (turretLimit1 == true) {
      turretSubsystem.turretDrive.set(ControlMode.PercentOutput, -.5);
      DriverStation.reportError("Limit Reached on turret, going back to safe position.", false);
    }
    
    if (turretLimit2 == true) {
      turretSubsystem.turretDrive.set(ControlMode.PercentOutput, .5);
      DriverStation.reportError("Limit Reached on turret, going back to safe position.", false);
    }
    
  }
}
