/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.elevatorSubsystem;


public class elevatorCommand extends CommandBase {

  public final elevatorSubsystem m_elevatorSubsystem;

  public elevatorCommand(elevatorSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevatorSubsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevatorSubsystem.setelevatorPID(0.1, 0, 0, 0);
    //m_shooterSubsystem.setShooterRPM(6380);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  System.out.println("Moving Elevator to Position.");    
    elevatorSubsystem.elevator1.set(ControlMode.Position, 2048);
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
