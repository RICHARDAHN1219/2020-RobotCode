/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class turretSubsystem extends SubsystemBase {
  /**
   * Creates a new turretSubsystem.
   */
  public static final TalonSRX turretDrive = new TalonSRX(Constants.Manipulator.TURRET_DRIVE);
  public static DigitalInput limit1 = new DigitalInput(8);
  public static DigitalInput limit2 = new DigitalInput(9);
  public turretSubsystem() {
    turretDrive.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 10);
    turretDrive.configForwardSoftLimitEnable(true);
    turretDrive.configReverseSoftLimitEnable(true);
    turretDrive.configForwardSoftLimitThreshold(
        (int) (Constants.Manipulator.kSoftMaxTurretAngle / (360.0 * Constants.Manipulator.kTurretRotationsPerTick)));
    turretDrive.configReverseSoftLimitThreshold(
        (int) (Constants.Manipulator.kSoftMinTurretAngle / (360.0 * Constants.Manipulator.kTurretRotationsPerTick)));
  }
  public static void turretHome(){
    if (turretDrive.getSelectedSensorPosition() < 0)
    {
      System.out.println("Turret is going clockwise to home");
      turretDrive.set(ControlMode.PercentOutput, .5);
    } else if (turretDrive.getSelectedSensorPosition() > 0)
    {
      System.out.println("Turret is going counter-clockwise home!");
      turretDrive.set(ControlMode.PercentOutput, -.5);
    } else if (turretDrive.getSelectedSensorPosition() == 0) {
      System.out.println("Turret is home!");
      turretDrive.set(ControlMode.PercentOutput, 0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  
  }
}
