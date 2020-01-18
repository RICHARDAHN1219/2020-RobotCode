/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.Constants.shooterConstants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class shooterSubsystem extends SubsystemBase {

  public static WPI_TalonFX shooter1 = new WPI_TalonFX(shooterConstants.shooter1);
  public static WPI_TalonFX shooter2 = new WPI_TalonFX(shooterConstants.shooter2);

  public shooterSubsystem() {
    shooter1.configFactoryDefault();
    shooter2.configFactoryDefault();
    shooter2.follow(shooter1);
    shooter2.setInverted(true);
    shooter1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, shooterConstants.shooterSlotIdx, shooterConstants.shooterTimeout);
    shooter1.setSensorPhase(true);
    shooter1.configNominalOutputForward(0, shooterConstants.shooterTimeout);
    shooter1.configNominalOutputReverse(0, shooterConstants.shooterTimeout);
    shooter1.configPeakOutputForward(1, shooterConstants.shooterTimeout);
    shooter1.configPeakOutputReverse(-1, shooterConstants.shooterTimeout);
    shooter1.setNeutralMode(NeutralMode.Coast);
    shooter2.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void periodic() {

  }

  public void setShooterRPM (double desiredRPM) {
    shooter1.set(ControlMode.Velocity, desiredRPM * 4096 / 600); //RPM must be less than 6380
  }

  public void setShooterPID (double P, double I, double D, double F) {
    shooter1.config_kP(shooterConstants.shooterSlotIdx, P, shooterConstants.shooterTimeout);
    shooter1.config_kI(shooterConstants.shooterSlotIdx, I, shooterConstants.shooterTimeout);
    shooter1.config_kD(shooterConstants.shooterSlotIdx, D, shooterConstants.shooterTimeout);
    shooter1.config_kF(shooterConstants.shooterSlotIdx, F, shooterConstants.shooterTimeout);
  }
}
