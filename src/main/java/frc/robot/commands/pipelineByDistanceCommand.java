/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.limelightSubsystem;

public class pipelineByDistanceCommand extends CommandBase {
  /**
   * Creates a new pipelineByDistanceCommand.
   */
  limelightSubsystem m_limelight;
  public pipelineByDistanceCommand(limelightSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    m_limelight = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Example output: currentDist = (98.5-24 / tan(10+20))
    //currentDist = 129.03 so 3x Zoom shall be used.
    double h1 = 24;
    double h2 = 98.5;
    double a1 = 10;
    double a2 = m_limelight.getTY();
    double oneXDist = 68;
    double twoXDist = 107;
    double threeXDist = 117;
    double currentDist = Math.abs(h2 - h1) / Math.tan(a1 + a2);
    if (currentDist >= oneXDist){
      m_limelight.set1xZoom();
      System.out.println("Switching to 1x Zoom");
    } else if (currentDist >= twoXDist){
      m_limelight.set2xZoom();
      System.out.println("Switching to 2x Zoom");
    } else if (currentDist >= threeXDist) {
      m_limelight.set3xZoom();
      System.out.println("Switching to 3x Zoom");
    }
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
