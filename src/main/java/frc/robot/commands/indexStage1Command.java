/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.indexerSubsystem;

public class indexStage1Command extends CommandBase {

  public indexStage1Command() {
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    indexerSubsystem.indexStage1_1.set(ControlMode.PercentOutput, 0.75);
  }

  @Override
  public void end(boolean interrupted) {
    indexerSubsystem.indexStage1_1.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

