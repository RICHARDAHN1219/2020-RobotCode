/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.elevatorSubsystem;

public class elevatorWinchCommand extends CommandBase {
  private elevatorSubsystem m_elevator;

  public elevatorWinchCommand(elevatorSubsystem elevator) {
    addRequirements(elevator);
    m_elevator = elevator;
  }

  @Override
  public void initialize() {
    m_elevator.setNeutralMode();
  }

  @Override
  public void execute() {
    //m_elevator.elevatorWinch.set(ControlMode.PercentOutput, 1);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
