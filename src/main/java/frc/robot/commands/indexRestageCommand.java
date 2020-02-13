/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.indexerSubsystem;

public class indexRestageCommand extends CommandBase {
  indexerSubsystem m_indexer;

  public indexRestageCommand(indexerSubsystem indexer) {
    addRequirements(indexer);
    m_indexer = indexer; 
  }

  @Override
  public void initialize() {
    //m_indexer.periodic = false;
    m_indexer.restageEndBallCount = m_indexer.ballCount;
  }


  // TODO: I may have broken this trying to make the indexer subsystem compatiple with commands. -Bryn

  @Override
  public void execute() {

    // TODO: I suggest putting this next block into the subsystem and calling that function.
    // make isFinished depend on ballReadyForIndexer and restageState.
    if (m_indexer.ballReadyForIndexer() == false && m_indexer.restageState == 0) {
      m_indexer.setIntakePercentOutput(-0.6);
      m_indexer.setBeltsPercentOutput(-1);
      m_indexer.setKickerPercentOutput(-0.3);
    }

    if (m_indexer.ballReadyForIndexer() == true && m_indexer.restageState == 0) {
      m_indexer.setIntakePercentOutput(0);
      m_indexer.setBeltsPercentOutput(0);
      m_indexer.setKickerPercentOutput(0);
      m_indexer.restageState = 1;
    }

    if (m_indexer.ballIndexed()== false && m_indexer.restageState == 1) {
      m_indexer.setIntakePercentOutput(0.6);
      m_indexer.setBeltsPercentOutput(1);
      m_indexer.setKickerPercentOutput(0.3);
    }

    if (!m_indexer.ballIndexed() == true && m_indexer.restageState == 1) {
      m_indexer.setIntakePercentOutput(0);
      m_indexer.setBeltsPercentOutput(0);
      m_indexer.setKickerPercentOutput(0);
      m_indexer.restageState = 2;
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_indexer.ballCount = m_indexer.restageEndBallCount;
    m_indexer.stateChangeCount = 2 * m_indexer.restageEndBallCount;
    //m_indexer.periodic = true;
  }

  @Override
  public boolean isFinished() {
    if (m_indexer.restageState == 2) {
      return true;
    }
    return false;
  }
}
