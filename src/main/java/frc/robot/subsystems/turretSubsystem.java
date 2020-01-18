/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class turretSubsystem extends SubsystemBase {
  /**
   * Creates a new turretSubsystem.
   */
  public static final TalonSRX turretDrive = new TalonSRX(Constants.Manipulator.TURRET_DRIVE);
  public turretSubsystem() {
    turretDrive.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
    turretDrive.configForwardSoftLimitEnable(true);
    turretDrive.configReverseSoftLimitEnable(true);
    turretDrive.configForwardSoftLimitThreshold(
        (int) (Constants.Manipulator.kSoftMaxTurretAngle / (360.0 * Constants.Manipulator.kTurretRotationsPerTick)));
    turretDrive.configReverseSoftLimitThreshold(
        (int) (Constants.Manipulator.kSoftMinTurretAngle / (360.0 * Constants.Manipulator.kTurretRotationsPerTick)));

    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  
  }
}
