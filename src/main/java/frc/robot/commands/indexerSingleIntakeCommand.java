/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.indexerSubsystem;

public class indexerSingleIntakeCommand extends CommandBase {

  private indexerSubsystem m_indexer;
  private int endStateChangeCount;

  public indexerSingleIntakeCommand(indexerSubsystem indexer) {
    addRequirements(indexer);
    m_indexer = indexer;
  }

  @Override
  public void initialize() {
    //TODO: we need to disable all the motor control in periodic for this to work.
    m_indexer.runOnlyIntake();
    final int endStateChangeCount = m_indexer.getStateChangeCount();
  }

  @Override
  public void execute() {
    //run indexer when a new ball appears
    if (m_indexer.ballReadyForIndexer() == true) {
      m_indexer.runIndexer();
    }

    /*
    //stop indexer when balls are properly staged
    else if (m_indexer.ballStaged() == true) {
      m_indexer.stop();
    }
    */

    //finish staging balls when this error state occurs
    else if (m_indexer.getStateChangeCount() != endStateChangeCount) {
      m_indexer.runIndexer();
    }

    /*
    //finish staging balls when this error state occurs
    else if (m_indexer.getBallCount() >= 1 && m_indexer.ballReadyForIndexer() == false && m_indexer.ballStaged() == false) {
      m_indexer.runIndexer();
    }
    */
    
    //stop indexer when balls are properly staged
    else if (m_indexer.ballStaged() == true && m_indexer.getStateChangeCount() == endStateChangeCount) {
      isFinished();
    }

    //prevent balls from exiting the indexer by accident
    else if (m_indexer.ballExiting() == true) {
      isFinished();
    }

    else {
      m_indexer.runOnlyIntake();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_indexer.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
