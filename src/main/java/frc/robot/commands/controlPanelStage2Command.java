/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.controlPanelSubsystem;


public class controlPanelStage2Command extends CommandBase {
  /**
   * Creates a new controlPanelStage2Command.
   */
  controlPanelSubsystem m_controlPanelSubsystem;
  String gameData;
  String currentColor = m_controlPanelSubsystem.getColor();

  public controlPanelStage2Command(controlPanelSubsystem cp) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_controlPanelSubsystem = cp;
    addRequirements (m_controlPanelSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_controlPanelSubsystem.setSpeed(0.2);
    String currentColor = m_controlPanelSubsystem.getColor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
}
      

 /*
  String gameData;
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    if (gameData.length() > 0) {
      switch (gameData.charAt(0)) {
      case 'B':
        // Blue case code
        break;
      case 'G':
        // Green case code
        break;
      case 'R':
        // Red case code
        break;
      case 'Y':
        // Yellow case code
        break;
      default:
        // This is corrupt data
        break;
      }
    } else {
      // Code for no data received yet
    }
*/
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
