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
  public double shooterRPM = 2500;
  private CANPIDController m_pidController;
  private CANEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

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
    neo_shooter2.follow(neo_shooter1);
    m_pidController = neo_shooter1.getPIDController();
    m_encoder = neo_shooter1.getEncoder();
    kP = 5e-5; 
    kI = 1e-6;
    kD = 0; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);


  }

  @Override
  public void periodic() {
    //SmartDashboard.putNumber("ShooterRPM", (int) (shooter1.getSelectedSensorVelocity() * 600 / 4096));
    SmartDashboard.putNumber("ShooterRPM", (int) (m_encoder.getVelocity() * 600 / 4096));
  }

  public void setShooterRPM (double desiredRPM) {
    //shooter1.set(ControlMode.Velocity, desiredRPM * 4096 / 600); //RPM must be less than 6380
    m_pidController.setReference(desiredRPM, ControlType.kVelocity);
  }

  /*public void setShooterPID (double P, double I, double D, double F) {
    shooter1.config_kP(shooterConstants.shooterSlotIdx, P, shooterConstants.shooterTimeout);
    shooter1.config_kI(shooterConstants.shooterSlotIdx, I, shooterConstants.shooterTimeout);
    shooter1.config_kD(shooterConstants.shooterSlotIdx, D, shooterConstants.shooterTimeout);
    shooter1.config_kF(shooterConstants.shooterSlotIdx, F, shooterConstants.shooterTimeout);
  }

  public void setPercentOutput(double percent) {
    shooter1.set(ControlMode.PercentOutput, percent);
  }

  public void disableCurrentLimit() {
    shooter1.configGetSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(false, 35, 35, 1));
  }

  public void enableCurrentLimit() {
   shooter1.configGetSupplyCurrentLimit(Robot.m_currentlimitMain);
}
*/

}
