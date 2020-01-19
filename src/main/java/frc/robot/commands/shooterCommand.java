/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.shooterSubsystem;

public class shooterCommand extends CommandBase {
  
  public final shooterSubsystem m_shooterSubsystem;

  public shooterCommand(shooterSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooterSubsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterSubsystem.setShooterPID(0.5, 0, 0, 0);
    m_shooterSubsystem.setShooterRPM(6380);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Spooling Flywheel");
    m_shooterSubsystem.setShooterRPM(6380);
    new WaitCommand(0.5);
    //Uose 3190 for testing purposes, as it is exactly half speed.
    //m_shooterSubsystem.setShooterRPM(1500);
    System.out.println("Starting Kicker Wheel");
    IndexerSubsystem.indexLoad.set(ControlMode.PercentOutput, .85);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Stopping Flywheel/Kicker Wheel");
    shooterSubsystem.shooter1.set(ControlMode.PercentOutput, 0);
    IndexerSubsystem.indexLoad.set(ControlMode.PercentOutput, 0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
