/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.indexerSubsystem;

public class indexStage1Command extends CommandBase {
  indexerSubsystem m_indexer;

  public indexStage1Command(indexerSubsystem indexer) {
    addRequirements(indexer);
    m_indexer = indexer;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    //indexStage1_1.set(ControlMode.PercentOutput, 0.75);
    m_indexer.setStage1PercentOutput(0.75);
  }

  @Override
  public void end(boolean interrupted) {
    //indexStage1_1.set(ControlMode.PercentOutput, 0);
    m_indexer.setStage1PercentOutput(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

