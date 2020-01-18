/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.turretSubsystem;

public class limelightTurretVisionCommand extends CommandBase {
  /**
   * Creates a new limelightTurretVisionCommand.
   */
  public limelightTurretVisionCommand(turretSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Robot.manualMode==false) {
       // These numbers must be tuned for Comp Robot!  Be careful!
    final double STEER_K = 0.05;                    // how hard to turn toward the target

    double tv = NetworkTableInstance.getDefault().getTable("limelight-one").getEntry("tv").getDouble(0);
    double tx = NetworkTableInstance.getDefault().getTable("limelight-one").getEntry("tx").getDouble(0);
    //boolean m_LimelightHasValidTarget = false;
    double m_LimelightSteerCommand = 0.0;

    if (tv < 1.0)
    {
     // m_LimelightHasValidTarget = false;
     // m_LimelightDriveCommand = 0.0;
      m_LimelightSteerCommand = 0.0;
      turretSubsystem.turretDrive.set(ControlMode.PercentOutput, 0.0);
      return;
    }

    //m_LimelightHasValidTarget = true;

    // Start with proportional steering
    double steer_cmd = tx * STEER_K;
    m_LimelightSteerCommand = steer_cmd;

    // try to drive forward until the target area reaches our desired area
    //double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

    // don't let the robot drive too fast into the goal
   /* if (drive_cmd > MAX_DRIVE)
    {
      drive_cmd = MAX_DRIVE;
    }
    m_LimelightDriveCommand = drive_cmd;
*/
    turretSubsystem.turretDrive.set(ControlMode.PercentOutput, m_LimelightSteerCommand);
    }
    else {
      turretSubsystem.turretDrive.set(ControlMode.PercentOutput, RobotContainer.m_operatorController.getX(Hand.kLeft));
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
