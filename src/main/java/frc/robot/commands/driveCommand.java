/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.driveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class driveCommand extends CommandBase {
  
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final driveSubsystem m_driveSubsystem;

  public driveCommand(driveSubsystem subsystem) {
    m_driveSubsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    //m_driveSubsystem.arcadeDrive(RobotContainer.m_driveController.getY(Hand.kLeft), RobotContainer.m_driveController.getX(Hand.kRight));
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
