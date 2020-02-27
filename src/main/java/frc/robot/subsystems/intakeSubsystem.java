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
import frc.robot.Constants.intakeConstants;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants.currentLimits;

public class intakeSubsystem extends SubsystemBase {

  private WPI_TalonSRX intake = new WPI_TalonSRX(intakeConstants.intakeMotor);
  public static Solenoid intakeSolenoid = new Solenoid(intakeConstants.intakeSolenoid);
    
  public intakeSubsystem() {
    intake.configSupplyCurrentLimit(currentLimits.m_currentlimitSecondary);
  }

  @Override
  public void periodic() {
  }
  
  public void setIntakePercentOutput(double percent) {
    intake.set(ControlMode.PercentOutput, percent);
  }

  public void deployIntake() {
    intakeSolenoid.set(true);
    setIntakePercentOutput(-0.9);
  }

  public void retractIntake() {
    intakeSolenoid.set(false);
    setIntakePercentOutput(0);
  }
}
