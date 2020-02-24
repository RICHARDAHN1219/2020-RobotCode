/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.fearxzombie.limelight;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.driveSubsystem;
import frc.robot.subsystems.shooterSubsystem;
import frc.robot.subsystems.turretSubsystem;

public class turretAutoTargeting extends CommandBase {
  private turretSubsystem m_turret;
  private Translation2d m_target;
  private driveSubsystem m_drive;
  private limelight m_limelight;
  private shooterSubsystem m_shooter;
  private double m_targetAngleDegrees = 0.0;
  private double m_errorDegrees = 0.0;
  double h1 = 0.6096;
  double h2 = 2.5019;
  double a1 = 32;

  // kP for limelight must be 1.0 or less
  private double kPlimelight = 0.5;

  /**
   * Creates a new turretAutoTargeting.
   * 
   * This keeps the turret aimed at a given target coordinate on the field using odometry to track
   * the current position and
   * 
   * @param target coordinates to aim at
   * @param turret Subsystem
   * @param drive  Subsystem
   * @param limelight Subsystem
   */
  public turretAutoTargeting(Translation2d target, turretSubsystem turret, driveSubsystem drive, limelight ll_util) {
    m_target = target;
    m_turret = turret;
    m_drive = drive;
    m_limelight = ll_util;
    // NOTE: do not add drive to addRequirements() or else we cannot drive while targeting
    // IMPORTANT: use READ ONLY methods from m_drive
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /**
     * Get the pose of robot in the near future. This anticipates where we need will need to be
     * aiming in the future, so we start turning the turret and will arleady be on target when we
     * get there. The slower we are moving the closer this pose is to our current pose.
     */
    Pose2d future_pose = m_drive.getFuturePose(0.1);

    // calculate m_targetAngleDegrees from robot to target
    m_targetAngleDegrees = angle2TargetDegrees(future_pose, m_target);
    SmartDashboard.putNumber("OdometryAngle2Target", m_targetAngleDegrees);

    // estimated distance to target based on odometery
    double dist_pose = distance2Target(future_pose, m_target);

    double ll_angleDegrees = 0;
    if ((int) m_limelight.getTV() == 1 ) {
      ll_angleDegrees = m_limelight.getTY();
      m_targetAngleDegrees = m_targetAngleDegrees + kPlimelight * ll_angleDegrees;
    }

    // move turret to angle
    m_turret.setAngleDegrees(m_targetAngleDegrees);

    m_errorDegrees = m_targetAngleDegrees - m_turret.getAngleDegrees();

    SmartDashboard.putNumber("Angle2Target", m_targetAngleDegrees);
    SmartDashboard.putNumber("Dist2Target", dist_pose);
    SmartDashboard.putNumber("LL_Dist2Target", m_limelight.getDist(h1, h2, a1));
    SmartDashboard.putNumber("AngleError", m_errorDegrees);
    SmartDashboard.putNumber("LL_Angle", ll_angleDegrees);
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


  // ------- Utility functions below ---------

  /**
   * Distance from robot to a given point on the field
   * 
   * @param pose     of the robot
   * @param location of the target
   * 
   * @return distance in meters
   **/
  public static double distance2Target(Pose2d robotPose, Translation2d target) {
    return robotPose.getTranslation().getDistance(target);
  }

  /**
   * Angle between robot pose and target
   * 
   * @param pose     of the robot
   * @param location of the target
   * 
   * @return angle to target in Radians
   **/
  public static double angle2TargetRadians(Pose2d robotPose, Translation2d target) {
    double robot_angle_to_field = robotPose.getRotation().getRadians();
    double angle_to_target = Math.atan2(robotPose.getTranslation().getX() - target.getX(),
        robotPose.getTranslation().getY() - target.getY());
    double theta = angle_to_target - robot_angle_to_field;

    // make sure resulting angle is between -pi and pi (or -180 and 180 degrees)
    if (theta < -Math.PI) {
      theta = theta + 2 * Math.PI;
    } else if (theta > Math.PI) {
      theta = theta - 2 * Math.PI;
    }

    return theta;
  }

  /**
   * Angle between robot pose and target
   * 
   * @param pose     of the robot
   * @param location of the target
   * 
   * @return angle to target in Degrees
   **/
  public static double angle2TargetDegrees(Pose2d robotPose, Translation2d target) {
    return Math.toDegrees(angle2TargetRadians(robotPose, target));
  }

}
