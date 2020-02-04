/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Robot;
import frc.robot.Constants.shooterConstants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

public class shooterSubsystem extends SubsystemBase {

  //private WPI_TalonFX shooter1 = new WPI_TalonFX(shooterConstants.shooter1);
  //private WPI_TalonFX shooter2 = new WPI_TalonFX(shooterConstants.shooter2);
  private CANSparkMax neo_shooter1 = new CANSparkMax(shooterConstants.shooter1, MotorType.kBrushless);
  private CANSparkMax neo_shooter2 = new CANSparkMax(shooterConstants.shooter2, MotorType.kBrushless);
  private CANPIDController m_pidController;
  private CANEncoder m_encoder;
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private double rpm;

  public shooterSubsystem() {
    /* shooter2.follow(shooter1);
    shooter2.setInverted(true);
    shooter1.configSupplyCurrentLimit(Robot.m_currentlimitMain);
    shooter2.configSupplyCurrentLimit(Robot.m_currentlimitMain);
    shooter1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, shooterConstants.shooterSlotIdx, shooterConstants.shooterTimeout);
    shooter1.setSensorPhase(true);
    shooter1.configNominalOutputForward(0, shooterConstants.shooterTimeout);
    shooter1.configNominalOutputReverse(0, shooterConstants.shooterTimeout);
    shooter1.configPeakOutputForward(1, shooterConstants.shooterTimeout);
    shooter1.configPeakOutputReverse(-1, shooterConstants.shooterTimeout);
    shooter1.setNeutralMode(NeutralMode.Coast);
    shooter2.setNeutralMode(NeutralMode.Coast);
    setShooterPID(0.1, 0, 0, 0); */
    neo_shooter1.setSmartCurrentLimit(35);
    neo_shooter2.setSmartCurrentLimit(35);
    neo_shooter2.setInverted(true);
    neo_shooter2.follow(neo_shooter1);
    m_pidController = neo_shooter1.getPIDController();
    m_encoder = neo_shooter1.getEncoder();
    kMaxOutput = 1; 
    kMinOutput = -1;
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);


  }

  @Override
  public void periodic() {
    //SmartDashboard.putNumber("ShooterRPM", (int) (shooter1.getSelectedSensorVelocity() * 600 / 4096));
    SmartDashboard.putNumber("ShooterRPM", (int) (m_encoder.getVelocity() * 600 / 4096));
    setShooterPID(0.1, 0, 0, 0);
    double rpm = SmartDashboard.getNumber("RPM", 0);
  }

  public void setShooterRPM (double desiredRPM) {
    //shooter1.set(ControlMode.Velocity, desiredRPM * 4096 / 600); //RPM must be less than 6380
    m_pidController.setReference(desiredRPM, ControlType.kVelocity);
  }
  public void testMode(){
    m_pidController.setReference(rpm, ControlType.kVelocity);
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
