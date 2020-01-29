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
import frc.robot.Robot;
import frc.robot.Constants.driveConstants;
public class driveSubsystem extends SubsystemBase {
  private WPI_TalonFX falcon1 = new WPI_TalonFX(driveConstants.falcon1);
  private WPI_TalonFX falcon2 = new WPI_TalonFX(driveConstants.falcon2);
  private WPI_TalonFX falcon3 = new WPI_TalonFX(driveConstants.falcon3);
  private WPI_TalonFX falcon4 = new WPI_TalonFX(driveConstants.falcon4);
  private PigeonIMU m_pigeon = new PigeonIMU(11);
  private static SpeedController leftSide;
  private static SpeedController rightSide;
  private DifferentialDrive drive;

  public driveSubsystem() {
    falcon1.configSupplyCurrentLimit(Robot.m_currentlimitMain);
    falcon2.configSupplyCurrentLimit(Robot.m_currentlimitMain);
    falcon2.configSupplyCurrentLimit(Robot.m_currentlimitMain);
    falcon4.configSupplyCurrentLimit(Robot.m_currentlimitMain);
    leftSide = new SpeedControllerGroup(falcon1, falcon3);
    rightSide = new SpeedControllerGroup(falcon2, falcon4);
    drive = new DifferentialDrive(leftSide, rightSide);
  }
  
  @Override
  public void periodic() {
  }
  
  public void arcadeDrive(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    drive.tankDrive(leftSpeed, rightSpeed);
  }
}
