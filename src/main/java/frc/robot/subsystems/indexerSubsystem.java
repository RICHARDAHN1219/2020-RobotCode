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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.indexConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//TODO work on conditions where we run or don't run the kicker motor
public class indexerSubsystem extends SubsystemBase {

  private WPI_TalonFX indexBelts = new WPI_TalonFX(indexConstants.indexBelts);
  private WPI_TalonFX indexKicker = new WPI_TalonFX(indexConstants.indexKicker);
  private DigitalInput Sensor1 = new DigitalInput(0);
  private DigitalInput Sensor2 = new DigitalInput(1);
  private DigitalInput Sensor3 = new DigitalInput(2);
  private boolean ballReady4IndexerLast = false;
  private boolean ballStagedLast = false;
  private boolean ballExitingLast = false;
  public boolean ballStaged;
  public boolean eject = false;
  private int stateChangeCount = 0;
  private int exitStateChangeCount = 0;
  public int ballCount = 0;

  public indexerSubsystem() {
    indexBelts.configSupplyCurrentLimit(Robot.m_currentlimitSecondary);
    indexKicker.configSupplyCurrentLimit(Robot.m_currentlimitSecondary);
    indexBelts.setNeutralMode(NeutralMode.Brake);
    indexKicker.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    boolean ballReady4Indexer = ! Sensor1.get();
    boolean ballStaged = ! Sensor2.get();
    boolean ballExiting = ! Sensor3.get();
    SmartDashboard.putNumber("ball count", ballCount);
    SmartDashboard.putNumber("state change count", stateChangeCount);
    
    //set the current limit to prevent jamming and unnecessary battery draw
    if (indexBelts.getSupplyCurrent() >= 20) {
      setBeltsPercentOutput(-0.5);
    }
    else {
    //prevent intake of new balls when we already have 5
    //EDIT: We are aiming for 4 balls by week 1 until 5 is figured out
    
    if (eject == true){
      setBeltsPercentOutput(1);
      setKickerPercentOutput(1);
    } 
    else {
    
    //move indexer when a new ball is ready to enter the system
    if (ballReady4Indexer == true) {
      setBeltsPercentOutput(1);
      setKickerPercentOutput(0.3);
    } 

    //stop indexer when balls are properly staged
    else if (ballStaged == true) {
      setBeltsPercentOutput(0);
    }

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
    
    //finish staging balls when this error state occurs
    if ((-1 + (ballCount * 2)) != stateChangeCount) {
      setBeltsPercentOutput(1);
      setKickerPercentOutput(0.3);
    }
    
    //finish staging balls when this error state occurs
    if ((ballCount >= 1) && ballReady4Indexer == false && ballStaged == false) {
      setBeltsPercentOutput(1);
      setKickerPercentOutput(0.3);
    }
    
    //automatically stage the balls for shooting when we have 4
    if (ballCount == 4 && ballExiting == false) {
      setBeltsPercentOutput(1);
    }
    
    //stop indexer when all 4 balls are staged for shooting
    else if (ballCount == 4 && ballExiting == true) {
      setBeltsPercentOutput(0);
    }
    if (ballExiting == true) {
      setBeltsPercentOutput(0);
      setKickerPercentOutput(0);
    }
  }
  
    //decrease ballCount as balls leave the indexer
    if (ballExiting != ballExitingLast && ballExiting == false) {
      ballCount -= 1;
      stateChangeCount = stateChangeCount - 2;
    }
    ballExitingLast = ballExiting;

    if (ballExiting != ballExitingLast) {
      exitStateChangeCount += 1;
      ballExitingLast = ballExiting;
    }
    
    if (stateChangeCount < 0) {
      stateChangeCount = 0;
    }

    if (ballCount == 0) {
      stateChangeCount = 0;
    }
  }
  }

  public void feedOneBall() {
    final int singleFeedInitialStateCount = exitStateChangeCount;
    final int singleFeedExitStateCount = singleFeedInitialStateCount + 2;

    if (exitStateChangeCount != singleFeedExitStateCount) {
      setBeltsPercentOutput(1);
      setKickerPercentOutput(1);
    }
    else {
      return;
    }
  }

  public void restageBalls() {
    final int restageInitialCount = stateChangeCount;
    final int restageState0FinishedCount = restageInitialCount + 2;
    final int restageState1FinishedCount = restageState0FinishedCount + 1;
    final int restageEndBallCount = ballCount;
    int restageState = 0;

    if (restageState == 0) {
      if (stateChangeCount != restageState0FinishedCount) {
      setBeltsPercentOutput(-1);
      setKickerPercentOutput(-0.3);
      }
    }
    else {
      restageState = 1;
    }

    if (restageState == 1) {
      if (stateChangeCount != restageState1FinishedCount) {
        setBeltsPercentOutput(1);
        setKickerPercentOutput(0.3);
      }
      else {
        ballCount = restageEndBallCount;
        stateChangeCount = -1 + (2 * restageEndBallCount);
        return;
      }
    }
  }

  public void setBeltsPercentOutput(double percent) {
    indexBelts.set(ControlMode.PercentOutput, percent);
  }
  
  public void setKickerPercentOutput(double percent) {
    indexKicker.set(ControlMode.PercentOutput, percent);
  }
}
