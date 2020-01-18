/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.elevatorConstants;

public class elevatorSubsystem extends SubsystemBase {
  /**
   * Creates a new elevatorSubsystem.
   */
  public static WPI_TalonFX elevator1 = new WPI_TalonFX(elevatorConstants.elevator1);
  public static WPI_TalonFX elevator2 = new WPI_TalonFX(elevatorConstants.elevator2);
  public static WPI_TalonFX elevatorWinch = new WPI_TalonFX(elevatorConstants.elevatorWinch);


  public elevatorSubsystem() {
    elevator1.configFactoryDefault();
    elevator2.configFactoryDefault();
    elevator2.follow(elevator1);
    elevator2.setInverted(true);
    elevator1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, elevatorConstants.elevatorSlotIdx, elevatorConstants.elevatorPivotTimeout);
    elevator1.setSensorPhase(true);
    elevator1.configNominalOutputForward(0, elevatorConstants.elevatorPivotTimeout);
    elevator1.configNominalOutputReverse(0, elevatorConstants.elevatorPivotTimeout);
    elevator1.configPeakOutputForward(1, elevatorConstants.elevatorPivotTimeout);
    elevator1.configPeakOutputReverse(-1, elevatorConstants.elevatorPivotTimeout);
    elevator1.setNeutralMode(NeutralMode.Coast);
    elevator2.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void periodic() {

  }

  public void setelevatorRPM (double desiredRPM) {
    elevator1.set(ControlMode.Velocity, desiredRPM * 4096 / 600); //RPM must be less than 6380
  }

  public void setelevatorPID (double P, double I, double D, double F) {
    elevator1.config_kP(elevatorConstants.elevatorSlotIdx, P, elevatorConstants.elevatorPivotTimeout);
    elevator1.config_kI(elevatorConstants.elevatorSlotIdx, I, elevatorConstants.elevatorPivotTimeout);
    elevator1.config_kD(elevatorConstants.elevatorSlotIdx, D, elevatorConstants.elevatorPivotTimeout);
    elevator1.config_kF(elevatorConstants.elevatorSlotIdx, F, elevatorConstants.elevatorPivotTimeout);
  }
}
