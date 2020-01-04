/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class driveSubsystem extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  //When referencing this code, use WPI_TalonFX as TalonFX does not work with DifferentialDrive.
   WPI_TalonFX falcon1 = new WPI_TalonFX(DriveConstants.FALCON_1);
   WPI_TalonFX falcon2 = new WPI_TalonFX(DriveConstants.FALCON_2);
   DifferentialDrive drive;

  public driveSubsystem() {
    drive = new DifferentialDrive(falcon1, falcon2);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void arcadeDrive(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    drive.tankDrive(leftSpeed, rightSpeed);
  }
}
