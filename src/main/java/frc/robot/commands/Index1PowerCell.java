/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.indexerSubsystem;

public class Index1PowerCell extends CommandBase {

  private indexerSubsystem m_indexer;
  private boolean running = false;

  /**
   * Creates a new Index1PowerCell.
   */
  public Index1PowerCell(indexerSubsystem indexer) {
    addRequirements(indexer);
    m_indexer = indexer;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // TODO: we need to disable all the motor control in periodic for this to work.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!running) {
      if (m_indexer.ballReadyForIndexer() && m_indexer.ballCount < 5) {
        m_indexer.runIntake();
        running = true;
      }
      //TODO: if ballcount >= 5 reverse indexer and eject the extra ball
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_indexer.ballIndexed()) {
        return true;
    }
    if (m_indexer.indexerFull()) {
        return true;
    }
    return false;
  }
}
