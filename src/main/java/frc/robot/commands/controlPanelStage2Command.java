/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.*;
import frc.robot.subsystems.colorSensorSubsystem;
import frc.robot.subsystems.controlPanelSubsystem;
import edu.wpi.first.wpilibj.DriverStation;


public class controlPanelStage2Command extends CommandBase {
  /**
   * Creates a new controlPanelStage2Command.
   */
  colorSensorSubsystem m_colorSensorSubsystem;
  controlPanelSubsystem m_controlPanelSubsystem;

  public controlPanelStage2Command(colorSensorSubsystem controlPanelSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_colorSensorSubsystem, m_controlPanelSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    String currentColor = m_colorSensorSubsystem.getColor();
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
    return false;
  }
}
