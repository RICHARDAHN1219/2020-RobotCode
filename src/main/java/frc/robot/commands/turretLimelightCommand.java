/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.limelightSubsystem;
import frc.robot.subsystems.shooterSubsystem;
import frc.robot.subsystems.turretSubsystem;

/**
 * turretLimelightCommand() - Deprecated. Use this only if turretAutoTargeting doesn't work.
 */
public class turretLimelightCommand extends CommandBase {
  turretSubsystem m_turret;
  limelightSubsystem m_limelight;
  shooterSubsystem m_shooter;

  public turretLimelightCommand(turretSubsystem subsystem1, limelightSubsystem subsystem2,
      shooterSubsystem subsystem3) {
    addRequirements(subsystem1);
    addRequirements(subsystem2);
    addRequirements(subsystem3);
    m_turret = subsystem1;
    m_limelight = subsystem2;
    m_shooter = subsystem3;
  }

  @Override
  public void initialize() {
    // m_limelight.setLEDMode(2);
    // m_limelight.setCAMMode(0);
  }

  @Override
  public void execute() {
    // These numbers must be tuned for Comp Robot! Be careful!
    final double STEER_K = 0.1; // how hard to turn toward the target
    double rpm = m_shooter.getRPMforDistanceMeter(m_limelight.getDist());
    double tv = m_limelight.getTV();;
    double tx = m_limelight.getTX();
    // boolean m_LimelightHasValidTarget = false;
    double m_LimelightSteerCommand = 0.0;

    if (tv < 1.0) {
      // m_LimelightHasValidTarget = false;
      // m_LimelightDriveCommand = 0.0;
      m_LimelightSteerCommand = 0.0;
      m_turret.setPercentOutput(RobotContainer.m_operatorController.getX(Hand.kLeft));
      return;
    }

    // m_LimelightHasValidTarget = true;
    // Start with proportional steering
    double steer_cmd = tx * STEER_K;
    m_LimelightSteerCommand = steer_cmd;

    // try to drive forward until the target area reaches our desired area
    // double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

    // don't let the robot drive too fast into the goal
    /*
     * if (drive_cmd > MAX_DRIVE) { drive_cmd = MAX_DRIVE; } m_LimelightDriveCommand = drive_cmd;
     */
    m_turret.setPercentOutput(m_LimelightSteerCommand);
    m_shooter.setShooterRPM(rpm);

  }

  @Override
  public void end(boolean interrupted) {
    // m_limelight.setLEDMode(1);
    // m_limelight.setCAMMode(1);
    m_turret.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
