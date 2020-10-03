/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooterSubsystem;

public class shooterHoodCommand extends CommandBase {
  
  shooterSubsystem m_shooter;

  public shooterHoodCommand(shooterSubsystem shooter) {
    addRequirements(shooter);
    m_shooter = shooter;
  }

  @Override
  public void initialize() {
    m_shooter.deployHood();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.retractHood();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
