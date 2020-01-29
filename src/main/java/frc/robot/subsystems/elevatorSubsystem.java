/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.elevatorConstants;

public class elevatorSubsystem extends SubsystemBase {

  private WPI_TalonFX elevatorWinch = new WPI_TalonFX(elevatorConstants.elevatorWinch);

  public elevatorSubsystem() {
    elevatorWinch.configSupplyCurrentLimit(Robot.m_currentlimitMain);
    elevatorWinch.setNeutralMode(NeutralMode.Coast);
  }

  public void setNeutralMode() {
    elevatorWinch.setNeutralMode(NeutralMode.Brake);    
  }
  
  
  @Override
  public void periodic() {
  }
}
