/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.turretSubsystem;

public class limelightTurretVisionCommand extends CommandBase {
  turretSubsystem m_turret;


  public limelightTurretVisionCommand(turretSubsystem subsystem) {
    addRequirements(subsystem);
    m_turret = subsystem;
  }

  @Override
  public void initialize() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(2);
  }

  @Override
  public void execute() {
    if (Robot.manualMode==false) {
      // These numbers must be tuned for Comp Robot!  Be careful!
      final double STEER_K = 0.07; //how hard to turn toward the target
      double tv = NetworkTableInstance.getDefault().getTable("limelight-one").getEntry("tv").getDouble(0);
      double tx = NetworkTableInstance.getDefault().getTable("limelight-one").getEntry("tx").getDouble(0);
      //boolean m_LimelightHasValidTarget = false;
      double m_LimelightSteerCommand = 0.0;

      if (tv < 1.0) {
        //m_LimelightHasValidTarget = false;
        //m_LimelightDriveCommand = 0.0;
        m_LimelightSteerCommand = 0.0;
        m_turret.setPercentOutput(RobotContainer.m_driveController.getX(Hand.kLeft) * 0.25);
        return;
      }

      //m_LimelightHasValidTarget = true;
      //Start with proportional steering
      double steer_cmd = tx * STEER_K;
      m_LimelightSteerCommand = steer_cmd;

      // try to drive forward until the target area reaches our desired area
      //double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

      // don't let the robot drive too fast into the goal
      /*
      if (drive_cmd > MAX_DRIVE) {
        drive_cmd = MAX_DRIVE;
      }
      m_LimelightDriveCommand = drive_cmd;
      */
      //m_turret.setPercentOutput(m_LimelightSteerCommand);
    }
    
    else if (Robot.manualMode = true) {
      m_turret.setPercentOutput(RobotContainer.m_driveController.getX(Hand.kLeft) * 0.25);
    }
    
    else if (Robot.turretHome = true){
      m_turret.turretHome();
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
