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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class indexerSubsystem extends SubsystemBase {

  public static WPI_TalonFX indexStage1_1 = new WPI_TalonFX(indexConstants.index1_1);
  public static WPI_TalonFX indexStage1_2 = new WPI_TalonFX(indexConstants.index1_2);
  public static WPI_TalonFX indexKicker = new WPI_TalonFX(indexConstants.indexKicker);
  public static DigitalInput Sensor1 = new DigitalInput(0);
  public static DigitalInput Sensor2 = new DigitalInput(1);
  public static DigitalInput Sensor3 = new DigitalInput(2);
  boolean sensor1Last = false;
  boolean ballStagedLast = false;
  boolean ballExitingLast = false;
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
    boolean ballReady4Indexer = ! Sensor1.get();
    boolean ballStaged = ! Sensor2.get();
    boolean ballExiting = ! Sensor3.get();
    SmartDashboard.putNumber("ball count", ballCount);
    SmartDashboard.putNumber("state change count", stateChangeCount);
    
    if (indexStage1_1.getSupplyCurrent() <= 20 || indexStage1_2.getSupplyCurrent() <= 20){
      indexStage1_1.set(ControlMode.PercentOutput, -0.5);
    }
    
    if (ballReady4Indexer == true) {
      indexStage1_1.set(ControlMode.PercentOutput, 0.75);
      indexKicker.set(ControlMode.PercentOutput, 0.75);
    } 
    
    else if (ballStaged == true) {
      indexStage1_1.set(ControlMode.PercentOutput, 0);
      indexKicker.set(ControlMode.PercentOutput, 0);
    }

    if (ballReady4Indexer != sensor1Last && ballReady4Indexer == true) {
      ballCount += 1;
      sensor1Last = ballReady4Indexer;
    }

    if (ballReady4Indexer != sensor1Last && ballReady4Indexer == false) {
      sensor1Last = ballReady4Indexer;
    }

    if (ballExiting != ballExitingLast && ballExiting == true) {
      ballCount -= 1;
      ballExitingLast = ballExiting;
    }

    if (ballExiting != ballExitingLast && ballExiting == false) {
      ballExitingLast = ballExiting;
    }

    if ((ballCount >= 1) && ballReady4Indexer == false && ballStaged == false) {
      indexStage1_1.set(ControlMode.PercentOutput, 0.75);
    }

    if (ballStaged != ballStagedLast && ballStaged == true) {
      stateChangeCount += 1;
      ballStagedLast = ballStaged;
    }

    if (ballStaged != ballStagedLast && ballStaged == false) {
      stateChangeCount += 1;
      ballStagedLast = ballStaged;
    }
    
    if (ballCount == 1 && stateChangeCount != 1) {
      indexStage1_1.set(ControlMode.PercentOutput, 0.75);
      indexKicker.set(ControlMode.PercentOutput, 0.75);
    }
    
    if (ballCount == 2 && stateChangeCount != 3) {
      indexStage1_1.set(ControlMode.PercentOutput, 0.75);
      indexKicker.set(ControlMode.PercentOutput, 0.75);
      if (ballCount == 2 && stateChangeCount == 3) {
        indexStage1_1.set(ControlMode.PercentOutput, 0);
        indexKicker.set(ControlMode.PercentOutput, 0);
      }
    }
    
    if (ballCount == 3 && stateChangeCount != 5) {
      indexStage1_1.set(ControlMode.PercentOutput, 0.75);
      indexKicker.set(ControlMode.PercentOutput, 0.75);
      if (ballCount == 3 && stateChangeCount == 5) {
        indexStage1_1.set(ControlMode.PercentOutput, 0);
        indexKicker.set(ControlMode.PercentOutput, 0);
      }
    }

    if (ballCount == 4 && stateChangeCount != 7) {
      indexStage1_1.set(ControlMode.PercentOutput, 0.75);
      indexKicker.set(ControlMode.PercentOutput, 0.75);
      if (ballCount == 4 && stateChangeCount == 7) {
        indexStage1_1.set(ControlMode.PercentOutput, 0);
        indexKicker.set(ControlMode.PercentOutput, 0);
      }
    }

    if (ballCount == 5 && stateChangeCount != 9) {
      indexStage1_1.set(ControlMode.PercentOutput, 0.75);
      indexKicker.set(ControlMode.PercentOutput, 0.75);
      if (ballCount == 5 && stateChangeCount == 9) {
        indexStage1_1.set(ControlMode.PercentOutput, 0);
        indexKicker.set(ControlMode.PercentOutput, 0);
      }
    }
  }
}
