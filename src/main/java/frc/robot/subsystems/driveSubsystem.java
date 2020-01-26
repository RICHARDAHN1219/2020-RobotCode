/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class driveSubsystem extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  //When referencing this code, use WPI_TalonFX as TalonFX is not for FRC use and does not work with DifferentialDrive.
  public static WPI_TalonFX falcon1 = new WPI_TalonFX(DriveConstants.FALCON_1);
  public static WPI_TalonFX falcon2 = new WPI_TalonFX(DriveConstants.FALCON_2);
  public static WPI_TalonFX falcon3 = new WPI_TalonFX(DriveConstants.FALCON_3);
  public static WPI_TalonFX falcon4 = new WPI_TalonFX(DriveConstants.FALCON_4);
   public PigeonIMU m_pigeon = new PigeonIMU(11);
   public static SpeedController leftSide;
   public static SpeedController rightSide;
   DifferentialDrive drive;

  public driveSubsystem() {
    leftSide = new SpeedControllerGroup(falcon1, falcon3);
    rightSide = new SpeedControllerGroup(falcon2, falcon4);
    drive = new DifferentialDrive(leftSide, rightSide);
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
