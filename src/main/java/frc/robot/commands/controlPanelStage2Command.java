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
  
  /* Trying to reference count, I ran into the problem that since count is a private
  integer on the controlPanelSubsystem, I can't access it from here. A bit confused on how to make that 
  happen, would this be the right way to do it because I have a feeling its not. */
  int count = m_controlPanelSubsystem.colorNumbers();
  

  public controlPanelStage2Command(controlPanelSubsystem cp) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_controlPanelSubsystem = cp;
    addRequirements(m_controlPanelSubsystem);
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
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    char currentColorChar = currentColor.charAt(0);
    String gameData = DriverStation.getInstance().getGameSpecificMessage();
    char stage2ColorChar = gameData.charAt(0);

     //TODO:Figure out what actions to take if one of these options isn't the case
    if (stage2ColorChar == 'B' && currentColorChar == 'B' && count == 2) {
      return true;
      // move counterclockwise 2
    }
    if (stage2ColorChar == 'B' && currentColorChar == 'Y' && count == 1) {
      return true;
      // move counterclockwise 1
    }
    if (stage2ColorChar == 'B' && currentColorChar == 'R' && count == 0) {
      return true;
      // don't move
    }
    if (stage2ColorChar == 'B' && currentColorChar == 'G' && count == -1) {
      return true;
      // move clockwise 1
    }

    if (stage2ColorChar == 'Y' && currentColorChar == 'B' && count == 1) {
      return true;
      // move counterclockwise 1
    }
    if (stage2ColorChar == 'Y' && currentColorChar == 'Y' && count == 2) {
      return true;
      // move counterclockwise 2
    }
    if (stage2ColorChar == 'Y' && currentColorChar == 'R' && count == -1) {
      return true;
      // move clockwise 1
    }
    if (stage2ColorChar == 'Y' && currentColorChar == 'G' && count == 0) {
      return true;
      // don't move
    }

    if (stage2ColorChar == 'R' && currentColorChar == 'R' && count == 2) {
      return true;
      // move counterclockwise 2
    }
    if (stage2ColorChar == 'R' && currentColorChar == 'Y' && count == 1) {
      return true;
      // move counterclockwise 1
    }
    if (stage2ColorChar == 'R' && currentColorChar == 'G' && count == -1) {
      return true;
      // move clockwise 1
    }
    if (stage2ColorChar == 'R' && currentColorChar == 'B' && count == 0) {
      return true;
      // don't move
    }

    if (stage2ColorChar == 'G' && currentColorChar == 'B' && count == -1) {
      return true;
      // move clockwise 1
    }
    if (stage2ColorChar == 'G' && currentColorChar == 'Y' && count == 0) {
      return true;
      // don't move
    }
    if (stage2ColorChar == 'G' && currentColorChar == 'R' && count == 1) {
      return true;
      // move counterclockwise 1
    }
    if (stage2ColorChar == 'G' && currentColorChar == 'G' && count == 2) {
      return true;
      // move counterclockwise 2
    }

    return false;
  }
}
