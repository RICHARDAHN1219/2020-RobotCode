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
  private double m_desiredRPM = 0;
  private boolean m_atSpeed = false;
  private long m_initalTime = 0;

  public shooterSubsystem() {

    neo_shooter1.restoreFactoryDefaults();
    neo_shooter2.restoreFactoryDefaults();

    //Current Limits for use on competition bot
    //neo_shooter1.setSmartCurrentLimit(35);
    //neo_shooter2.setSmartCurrentLimit(35);
    
    // Set coast mode
    neo_shooter1.setIdleMode(CANSparkMax.IdleMode.kCoast);
    neo_shooter2.setIdleMode(CANSparkMax.IdleMode.kCoast);

    neo_shooter2.follow(neo_shooter1, true);
    m_pidController = neo_shooter1.getPIDController();
    m_encoder = neo_shooter1.getEncoder(EncoderType.kHallSensor, 4096);
    kMaxOutput = 1; 
    kMinOutput = -1;
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    setShooterPID(0.0003, 0.00000025, 0, 0.0002, 250);

    SmartDashboard.putNumber("ShooterRPM", m_desiredRPM);
    SmartDashboard.putNumber("UpdatedRPM", -1);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ActualShooterRPM", (int) m_encoder.getVelocity());
    //m_desiredRPM = SmartDashboard.getNumber("DesiredShooterRPM", 0);
    double rpm = SmartDashboard.getNumber("ShooterRPM", -1);
    if (rpm != -1) {
      if (m_desiredRPM != rpm ) {
        setShooterRPM(rpm);
        System.out.println("New shooter desired RPM: "  + m_desiredRPM);
        // lets' confirm we're changing this
        SmartDashboard.putNumber("UpdatedRPM", m_desiredRPM);
        m_initalTime = System.nanoTime();
        m_atSpeed = false;
      }
    }

    m_initalTime = System.nanoTime();
    if (isAtSpeed()) {
      if (!m_atSpeed) {
        SmartDashboard.putNumber("Time2RPM", System.nanoTime() - m_initalTime);
      }
      m_atSpeed = true;
    }
    else {
      m_atSpeed = false;
      m_initalTime = System.nanoTime();
    }
    SmartDashboard.putBoolean("isAtSpeed", m_atSpeed);
  }

  public void setShooterRPM (double desiredRPM) {
    m_desiredRPM = desiredRPM;
    m_pidController.setReference(desiredRPM, ControlType.kVelocity);
  }

  public void testMode(){
    //m_desiredRPM = SmartDashboard.getNumber("DesiredShooterRPM", 0);
    //System.out.println("Shooter desired ROM: "  + m_desiredRPM);
    //m_pidController.setReference(m_desiredRPM, ControlType.kVelocity);
    //System.out.println("Activating Test Mode");
  }

  public void setShooterPID (double P, double I, double D, double F, double iZ) {
    m_pidController.setP(P);
    m_pidController.setI(I);
    m_pidController.setD(D);
    m_pidController.setFF(F);
    m_pidController.setIZone(iZ);
  }

  //Current limiting on the fly switching removed due to the SparkMAX API not supporting that sort of switch.
  public void setPercentOutput(double percent) {
    neo_shooter1.set(percent);
  }
  
  public boolean isAtSpeed(){
    double error = m_desiredRPM - m_encoder.getVelocity();
    SmartDashboard.putNumber("RPM_Error", error);
    if (Math.abs(error) < 100){
      return true;
    } else {
      return false;
    }
  }
  public void stop() {
    setPercentOutput(0.0);
  }
}
