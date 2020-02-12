/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.controlPanelSubsystem;
import edu.wpi.first.wpilibj.DriverStation;

public class controlPanelStage2Command extends CommandBase {
  /**
   * Creates a new controlPanelStage2Command.
   */
  controlPanelSubsystem m_controlPanelSubsystem;
  String gameData;
  String currentColor = m_controlPanelSubsystem.getColor();
  int count = 0;

  public controlPanelStage2Command(controlPanelSubsystem cp) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_controlPanelSubsystem = cp;
    addRequirements(m_controlPanelSubsystem);

    // Before we start, reset the count so old state from other stages or previous attempts
    // don't mess up our count.
    m_controlPanelSubsystem.resetColorCount();

    // use a public get method to make private variables visible to other classes
    count = m_controlPanelSubsystem.getColorCount();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_controlPanelSubsystem.setSpeed(0.2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_controlPanelSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_controlPanelSubsystem.moveToGamePosition() == true){
      return true;
    }
    return false;
  }
}
