/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.indexConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class indexerSubsystem extends SubsystemBase {

  public static WPI_TalonFX indexStage1_1 = new WPI_TalonFX(indexConstants.index1_1);
  public static WPI_TalonFX indexStage1_2 = new WPI_TalonFX(indexConstants.index1_2);
  public static WPI_TalonFX indexKicker = new WPI_TalonFX(indexConstants.indexKicker);
  public static DigitalInput Sensor1 = new DigitalInput(0);
  public static DigitalInput Sensor2 = new DigitalInput(1);
  public static DigitalInput Sensor3 = new DigitalInput(2);
  boolean sensor1Last = true;
  boolean sensor2Last = true;
  boolean sensor3Last = true;
  int stateChangeCount = 0;
  int ballCount = 0;

  public indexerSubsystem() {
    indexStage1_2.follow(indexStage1_1);
    indexStage1_1.setInverted(true);
    indexStage1_1.configSupplyCurrentLimit(Robot.m_currentlimitSecondary);
    indexStage1_2.configSupplyCurrentLimit(Robot.m_currentlimitSecondary);
    indexKicker.configSupplyCurrentLimit(Robot.m_currentlimitSecondary);
  }

  @Override
  public void periodic() {
    boolean sensor1 = Sensor1.get();
    boolean sensor2 = Sensor2.get();
    boolean sensor3 = Sensor3.get();
    SmartDashboard.putNumber("ball count", ballCount);
    SmartDashboard.putNumber("state change count", stateChangeCount);
    
    if (indexStage1_1.getSupplyCurrent() <= 20 || indexStage1_2.getSupplyCurrent() <= 20){
      indexStage1_1.set(ControlMode.PercentOutput, -0.5);
    }
    
    if (sensor1 == false) {
      indexerSubsystem.indexStage1_1.set(ControlMode.PercentOutput, 0.75);
      indexerSubsystem.indexKicker.set(ControlMode.PercentOutput, 0.75);
    } 
    
    else if (sensor2 == false) {
      indexerSubsystem.indexStage1_1.set(ControlMode.PercentOutput, 0);
      indexerSubsystem.indexKicker.set(ControlMode.PercentOutput, 0);
    }

    if (sensor1 != sensor1Last && sensor1 == false) {
      ballCount += 1;
      sensor1Last = sensor1;
    }

    if (sensor1 != sensor1Last && sensor1 == true) {
      sensor1Last = sensor1;
    }

    if (sensor3 != sensor3Last && sensor3 == false) {
      ballCount -= 1;
      sensor3Last = sensor3;
    }

    if (sensor3 != sensor3Last && sensor3 == true) {
      sensor3Last = sensor3;
    }

    if ((ballCount >= 1) && sensor1 == true && sensor2 == true) {
      indexerSubsystem.indexStage1_1.set(ControlMode.PercentOutput, 0.75);
    }

    if (sensor2 != sensor2Last && sensor2 == false) {
      stateChangeCount += 1;
      sensor2Last = sensor2;
    }

    if (sensor2 != sensor2Last && sensor2 == true) {
      stateChangeCount += 1;
      sensor2Last = sensor2;
    }
    
    if (ballCount == 1 && stateChangeCount != 1) {
      indexerSubsystem.indexStage1_1.set(ControlMode.PercentOutput, 0.75);
      indexerSubsystem.indexKicker.set(ControlMode.PercentOutput, 0.75);
    }
    
    if (ballCount == 2 && stateChangeCount != 3) {
      indexerSubsystem.indexStage1_1.set(ControlMode.PercentOutput, 0.75);
      indexerSubsystem.indexKicker.set(ControlMode.PercentOutput, 0.75);
      if (ballCount == 2 && stateChangeCount == 3) {
        indexerSubsystem.indexStage1_1.set(ControlMode.PercentOutput, 0);
        indexerSubsystem.indexKicker.set(ControlMode.PercentOutput, 0);
      }
    }
    
    if (ballCount == 3 && stateChangeCount != 5) {
      indexerSubsystem.indexStage1_1.set(ControlMode.PercentOutput, 0.75);
      indexerSubsystem.indexKicker.set(ControlMode.PercentOutput, 0.75);
      if (ballCount == 3 && stateChangeCount == 5) {
        indexerSubsystem.indexStage1_1.set(ControlMode.PercentOutput, 0);
        indexerSubsystem.indexKicker.set(ControlMode.PercentOutput, 0);
      }
    }

    if (ballCount == 4 && stateChangeCount != 7) {
      indexerSubsystem.indexStage1_1.set(ControlMode.PercentOutput, 0.75);
      indexerSubsystem.indexKicker.set(ControlMode.PercentOutput, 0.75);
      if (ballCount == 4 && stateChangeCount == 7) {
        indexerSubsystem.indexStage1_1.set(ControlMode.PercentOutput, 0);
        indexerSubsystem.indexKicker.set(ControlMode.PercentOutput, 0);
      }
    }

    if (ballCount == 5 && stateChangeCount != 9) {
      indexerSubsystem.indexStage1_1.set(ControlMode.PercentOutput, 0.75);
      indexerSubsystem.indexKicker.set(ControlMode.PercentOutput, 0.75);
      if (ballCount == 5 && stateChangeCount == 9) {
        indexerSubsystem.indexStage1_1.set(ControlMode.PercentOutput, 0);
        indexerSubsystem.indexKicker.set(ControlMode.PercentOutput, 0);
      }
    }
  }
}
