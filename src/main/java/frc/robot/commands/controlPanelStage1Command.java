/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.colorSensorSubsystem;
import frc.robot.subsystems.controlPanelSubsystem;

public class controlPanelStage1Command extends CommandBase {
  /**
   * Creates a new controlPanelStage1Command.
   */
  private colorSensorSubsystem m_colorSensorSubsystem;
  private controlPanelSubsystem m_controlPanelSubsystem;

  public controlPanelStage1Command(controlPanelSubsystem colorWheelSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
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
    if (m_colorSensorSubsystem.getRotationCount() >= 3) {
      return true;
    }

    return false;
  }
}
