/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turretSubsystem;

public class turretAutoTargeting extends CommandBase {
  private turretSubsystem m_turret;
  private Translation2d   m_target;
  private double m_targetAngleDegrees = 0.0;
  private double m_errorDegrees = 0.0;

  /**
   * Creates a new turretAutoTargeting.
   */
  public turretAutoTargeting(Translation2d target, turretSubsystem turret) {
    m_target = target;
    m_turret = turret;
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_errorDegrees = (m_turret.getAngleDegrees() - m_targetAngleDegrees);
    SmartDashboard.putNumber("TargetAngle Error", m_errorDegrees);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO: get robot pose
    // TODO: get robot velocity and direction
    // TODO: calculate position at time x (where x is how long it takes to shoot) in the future
    // TODO: calculate m_targetAngleDegrees from robot to target
    m_targetAngleDegrees = 0;
    // TODO: move turret to angle
    m_turret.setAngleDegrees(m_targetAngleDegrees);
  }

  /** 
   * onTarget - returns true if turret is at it's designated angle
   * 
   * @param double tolerence in degrees
   * 
   * @return boolean
   */
  public boolean onTarget(double toleranceDeg) {
    if (Math.abs(m_errorDegrees) < toleranceDeg) {
      return true;
    }
    return false;
  }

  // default to one degree error
  public boolean onTarget() {
    // 1 degree of error means we are off by 1.7cm for every meter we are from the target
    // 1 degree error means missing the goal at 21m (69 feet)
    return onTarget(1.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
