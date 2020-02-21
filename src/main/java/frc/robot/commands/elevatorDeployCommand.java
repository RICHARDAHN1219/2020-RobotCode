/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.elevatorSubsystem;

public class elevatorDeployCommand extends CommandBase {

  elevatorSubsystem m_elevator;

  public elevatorDeployCommand(elevatorSubsystem elevator) {
    addRequirements(elevator);
    m_elevator = elevator;
  }

  @Override
  public void initialize() {
    if (m_elevator.getElevatorDeployed() == true) {
      m_elevator.retractElevator();
    }
    if (m_elevator.getElevatorDeployed() == false) {
      m_elevator.deployElevator();
    }
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    if (m_elevator.getElevatorDeployed() == true) {
      m_elevator.setElevatorDeployed(false);
    }
    if (m_elevator.getElevatorDeployed() == false) {
      m_elevator.setElevatorDeployed(true);
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
