/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.intakeConstants;

public class intakeSubsystem extends SubsystemBase {

  public WPI_TalonSRX intake = new WPI_TalonSRX(intakeConstants.intakeMotor);
  
  public intakeSubsystem() {
    intake.configSupplyCurrentLimit(Robot.m_currentlimitSecondary);
  }

  @Override
  public void periodic() {
    double current = intake.getSupplyCurrent();
    if (current >= 20) {
      intake.set(ControlMode.PercentOutput, -0.5);   
    }
    setIntakePercentOutput(0.75);
  }
  public void setIntakePercentOutput(double percent) {
    intake.set(ControlMode.PercentOutput, percent);
  }
}
