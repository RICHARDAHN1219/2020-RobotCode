/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class manualMode extends CommandBase {

  public manualMode() {
  }

  @Override
  public void initialize() {
    if (Robot.manualMode=false){
      System.out.println("Overriding Limelight Control");
      Robot.manualMode = true;
    } 
    else {
      Robot.manualMode = false;
    }
  }

  @Override
  public void execute() {
  }
  
  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
