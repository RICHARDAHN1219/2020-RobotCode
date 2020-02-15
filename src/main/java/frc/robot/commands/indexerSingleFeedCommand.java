/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.indexerSubsystem;

public class indexerSingleFeedCommand extends CommandBase {

  private indexerSubsystem m_indexer;
  private int endStateChangeCount;

  public indexerSingleFeedCommand(indexerSubsystem indexer) {
    addRequirements(indexer);
    m_indexer = indexer;
  }

  @Override
  public void initialize() {
    endStateChangeCount = m_indexer.getExitStateChangeCount() + 2;
  }

  @Override
  public void execute() {
    if (m_indexer.getExitStateChangeCount() < endStateChangeCount && m_indexer.ballExiting() != true) {
      m_indexer.ejectIndexer();
    }
    else {
      isFinished();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_indexer.stopIndexer();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
