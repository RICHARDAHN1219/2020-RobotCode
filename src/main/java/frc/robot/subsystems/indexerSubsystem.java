/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.indexConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.GenericHID.Hand;

//TODO work on conditions where we run or don't run the kicker motor
public class indexerSubsystem extends SubsystemBase {

  private WPI_TalonSRX indexIntake = new WPI_TalonSRX(indexConstants.indexIntake);
  private WPI_TalonFX indexBelts = new WPI_TalonFX(indexConstants.indexBelts);
  private WPI_TalonFX indexKicker = new WPI_TalonFX(indexConstants.indexKicker);
  public DigitalInput Sensor1 = new DigitalInput(0);
  public DigitalInput Sensor2 = new DigitalInput(1);
  private DigitalInput Sensor3 = new DigitalInput(2);
  private boolean ballReady4IndexerLast = false;
  private boolean ballStagedLast = false;
  private boolean ballExitingLast = false;
  public boolean ballReady4Indexer;
  public boolean ballStaged;
  public boolean eject = false;
  public int stateChangeCount = 0;
  private int exitStateChangeCount = 0;
  public int ballCount = 0;
  public int restageState = 0;
  public boolean periodic = true;
  public int restageEndBallCount;

  public indexerSubsystem() {
    indexBelts.configSupplyCurrentLimit(Robot.m_currentlimitSecondary);
    indexKicker.configSupplyCurrentLimit(Robot.m_currentlimitSecondary);
    indexIntake.configSupplyCurrentLimit(Robot.m_currentlimitSecondary);
    indexBelts.setNeutralMode(NeutralMode.Brake);
    indexKicker.setNeutralMode(NeutralMode.Brake);
    indexIntake.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    boolean ballReady4Indexer = ! Sensor1.get();
    boolean ballStaged = ! Sensor2.get();
    boolean ballExiting = ! Sensor3.get();
    SmartDashboard.putNumber("ball count", ballCount);
    SmartDashboard.putNumber("state change count", stateChangeCount);
    SmartDashboard.putNumber("restage state", restageState);
    
    //prevent intake of new balls when we already have 5
    //EDIT: We are aiming for 4 balls by week 1 until 5 is figured out
    if (periodic == true) {
    if (Robot.manualMode == true) {
      setIntakePercentOutput((RobotContainer.m_operatorController.getTriggerAxis(Hand.kRight) - RobotContainer.m_operatorController.getTriggerAxis(Hand.kLeft)) * 0.6);
      setBeltsPercentOutput(RobotContainer.m_operatorController.getTriggerAxis(Hand.kRight) - RobotContainer.m_operatorController.getTriggerAxis(Hand.kLeft));
      setKickerPercentOutput(RobotContainer.m_operatorController.getTriggerAxis(Hand.kRight) - RobotContainer.m_operatorController.getTriggerAxis(Hand.kLeft));
    }
    else {

    //move indexer when a new ball is ready to enter the system
    if (ballReady4Indexer == true) {
      setIntakePercentOutput(0.6);
      setBeltsPercentOutput(1);
      setKickerPercentOutput(0.3);
    } 

    //stop indexer when balls are properly staged
    else if (ballStaged == true) {
      setBeltsPercentOutput(0);
    }

    //finish staging balls when this error state occurs
    if ((-1 + (ballCount * 2)) != stateChangeCount) {
      setIntakePercentOutput(0.6);
      setBeltsPercentOutput(1);
      setKickerPercentOutput(0.3);
    }
    
    //finish staging balls when this error state occurs
    if ((ballCount >= 1) && ballReady4Indexer == false && ballStaged == false) {
      setIntakePercentOutput(0.6);
      setBeltsPercentOutput(1);
      setKickerPercentOutput(0.3);
    }
    
    //automatically stage the balls for shooting when we have 4
    if (ballCount == 4 && ballExiting == false) {
      setIntakePercentOutput(0.6);
      setBeltsPercentOutput(1);
    }
    
    //stop indexer when all 4 balls are staged for shooting
    else if (ballCount == 4 && ballExiting == true) {
      setIntakePercentOutput(0);
      setBeltsPercentOutput(0);
    }

    if (ballExiting == true) {
      setIntakePercentOutput(0);
      setBeltsPercentOutput(0);
      setKickerPercentOutput(0);
      
    if (eject == true){
      setBeltsPercentOutput(1);
      setKickerPercentOutput(1);
      setIntakePercentOutput(0.6);
    }
    }
    }
    }
    else {
    
    //increase ball count as balls enter the indexer
    if (ballReady4Indexer != ballReady4IndexerLast && ballReady4Indexer == true) {
      ballCount += 1;  
    }
    ballReady4IndexerLast = ballReady4Indexer;
    
    //count number of state changes on ballStaged sensor to combat error states
    if (ballStaged != ballStagedLast) {
      stateChangeCount += 1;
      ballStagedLast = ballStaged;
    }
  }
    
    //decrease ballCount as balls leave the indexer
    if (ballExiting != ballExitingLast && ballExiting == false) {
      ballCount -= 1;
      stateChangeCount = stateChangeCount - 2;
    }
    ballExitingLast = ballExiting;

    //count number of state changes as balls leave the system
    if (ballExiting != ballExitingLast) {
      exitStateChangeCount += 1;
      ballExitingLast = ballExiting;
    }
    
    //don't let state changes go below zero
    if (stateChangeCount < 0) {
      stateChangeCount = 0;
    }

    //can't have negative balls in the robot
    if (ballCount == 0) {
      stateChangeCount = 0;
    }
  }

  public void feedOneBall() {
    final int singleFeedInitialStateCount = exitStateChangeCount;
    final int singleFeedExitStateCount = singleFeedInitialStateCount + 2;

    if (exitStateChangeCount != singleFeedExitStateCount) {
      setIntakePercentOutput(0.6);
      setBeltsPercentOutput(1);
      setKickerPercentOutput(1);
    }
    else {
      return;
    }
  }

  public void setBeltsPercentOutput(double percent) {
    indexBelts.set(ControlMode.PercentOutput, percent);
  }
  
  public void setKickerPercentOutput(double percent) {
    indexKicker.set(ControlMode.PercentOutput, percent);
  }

  public void setIntakePercentOutput(double percent) {
    indexIntake.set(ControlMode.PercentOutput, percent);
  }
}