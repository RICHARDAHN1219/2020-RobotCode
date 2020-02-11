/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.shooterConstants;

public class shooterSubsystem extends SubsystemBase {

  private CANSparkMax neo_shooter1 = new CANSparkMax(shooterConstants.shooter1, MotorType.kBrushless);
  private CANSparkMax neo_shooter2 = new CANSparkMax(shooterConstants.shooter2, MotorType.kBrushless);
  private CANPIDController m_pidController;
  private CANEncoder m_encoder;
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private double rpm;

  public shooterSubsystem() {
    //neo_shooter1.setSmartCurrentLimit(35);
    //neo_shooter2.setSmartCurrentLimit(35);
    neo_shooter2.follow(neo_shooter1, true);
    m_pidController = neo_shooter1.getPIDController();
    m_encoder = neo_shooter1.getEncoder(EncoderType.kHallSensor, 2048);
    kMaxOutput = 1; 
    kMinOutput = -1;
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);
    SmartDashboard.putNumber("RPM", rpm);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ShooterRPM", (int) (m_encoder.getVelocity() * 600 / 4096));
    setShooterPID(5e-5, 1e-6, 0, 0);
  }

  public void setShooterRPM (double desiredRPM) {
    m_pidController.setReference(desiredRPM, ControlType.kVelocity);
  }
  
  public void testMode(){
    double rpm = SmartDashboard.getNumber("RPM", 0);
    System.out.println(rpm);
    m_pidController.setReference(rpm, ControlType.kVelocity);
    System.out.println("Activating Test Mode");
  }

  public void setShooterPID (double P, double I, double D, double F) {
    m_pidController.setP(P);
    m_pidController.setI(I);
    m_pidController.setD(D);
    m_pidController.setFF(F);
  }

  //Current limiting on the fly switching removed due to the SparkMAX API not supporting that sort of switch.
  public void setPercentOutput(double percent) {
    neo_shooter1.set(percent);
  }
}
