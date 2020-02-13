/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.indexerSubsystem;

public class indexerRestageCommand extends CommandBase {
  
  indexerSubsystem m_indexer;
  private int restageEndBallCount;

  public indexerRestageCommand(indexerSubsystem indexer) {
    addRequirements(indexer);
    m_indexer = indexer; 
  }

  @Override
  public void initialize() {
    //m_indexer.periodic = false;
    m_indexer.setRestageState(0);
    final int restageEndBallCount = m_indexer.getBallCount();
  }


  // TODO: I may have broken this trying to make the indexer subsystem compatiple with commands. -Bryn

  @Override
  public void execute() {

    // TODO: I suggest putting this next block into the subsystem and calling that function.
    // make isFinished depend on ballReadyForIndexer and restageState.
    if (m_indexer.ballReadyForIndexer() == false && m_indexer.getRestageState() == 0) {
      m_indexer.setIntakePercentOutput(-0.6);
      m_indexer.setBeltsPercentOutput(-1);
      m_indexer.setKickerPercentOutput(-0.3);
    }

    if (m_indexer.ballReadyForIndexer() == true && m_indexer.getRestageState() == 0) {
      m_indexer.setIntakePercentOutput(0);
      m_indexer.setBeltsPercentOutput(0);
      m_indexer.setKickerPercentOutput(0);
      m_indexer.setRestageState(1);
    }

    if (m_indexer.ballStaged() == false && m_indexer.getRestageState() == 1) {
      m_indexer.setIntakePercentOutput(0.6);
      m_indexer.setBeltsPercentOutput(1);
      m_indexer.setKickerPercentOutput(0.3);
    }

    if (!m_indexer.ballStaged() == true && m_indexer.getRestageState() == 1) {
      m_indexer.setIntakePercentOutput(0);
      m_indexer.setBeltsPercentOutput(0);
      m_indexer.setKickerPercentOutput(0);
      m_indexer.setRestageState(2);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_indexer.setBallCount(restageEndBallCount);
    m_indexer.setStateChangeCount(2 * restageEndBallCount);
  }

  @Override
  public boolean isFinished() {
    if (m_indexer.getRestageState() == 2) {
      return true;
    }
    return false;
  }
}
