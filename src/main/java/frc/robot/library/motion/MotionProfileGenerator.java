/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.library.motion;
//import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.driveSubsystem;
import frc.robot.subsystems.turretSubsystem;
import frc.robot.Constants;
import frc.robot.library.Util.Util;
//import frc.robot.Constants.TrajectoryConstants;
//import frc.robot.commands.limelightTurretVisionCommand;
//import edu.wpi.first.wpilibj.controller.RamseteController;
import frc.robot.commands.limelightTurretVisionCommand;
import frc.robot.RobotContainer;
import frc.robot.library.vision.limelight;




public class MotionProfileGenerator extends CommandBase {
  public static Translation2d m_translation;
  public static Rotation2d m_rotation;
  /**
   * Uses setPoint Generator to generate an angle we are currently at above the
   * x-axis of the field. We calculate the angle between the horizon and a line
   * determining the distance it takes for the robot to travel from its current
   * point to the new point on the field we want to get to.
   */
  public MotionProfileGenerator() {

  }
  // Called when the command is initially scheduled.



/**Use method createPosePath in order to be able to use the Pose2d to generate a path under certain conditions.
 * Either move the robot forward while adjusting rotation with turret in initial position
 * OR
 * Rotate the robot with the turret in its initial position
 * 
 * First, have the robot flash the LL to the vision target to generate path
 */
 public void createPosePath() {
    limelight.getDist(0.6096, 2.5019,32);
    if (Util.angleCorrection >= Math.toDegrees(45) && Robot.turretHome == true && Util.distanceToObjective() >= 2.5) {
        //TrajectoryConstants.m_x = deltaOverall_x;
        //TrajectoryConstants.m_y = deltaOverall_y;
      //rotate the robot itself with the characterization
      //proceed to the calculated distanceToObjective
      //this.Pose2d.Pose2d(Translation2d.Translation2d((double)deltaOverall_x, (double)deltaOverall_y), Rotation2d.fromDegrees(angleToObjective));
      Rotation2d.fromDegrees(Util.angleCorrection);
      new Pose2d (Util.deltaOverall_x, Util.deltaOverall_y, m_rotation); {//Pose2d(deltaOverall_x, deltaOverall_y, Rotation2d rotation);
        m_rotation = new Rotation2d(Util.angleToObjective);
        m_translation = new Translation2d(Util.deltaOverall_x, Util.deltaOverall_y); 
      }
      if (Util.angleCorrection() < Math.toDegrees(30)){//adjust angleCorrection later on
        RobotContainer.m_driveSubsystem.arcadeDrive(0, 0);
        Util.angleCorrection =  (Constants.turretConstants.kSoftMaxTurretAngle / (360.0 * Constants.turretConstants.kTurretRotationsPerTick));
        //
        //limelightTurretVisionCommand
      }

      //Use the turret to rotate in order lock onto the vision target from here

      //Translation2d.getDistance(Translation2d other);

      //Pose2d(Translation2d translation, Rotation2d rotation) is the structure
    }
    else if (Robot.turretHome == true  && Util.distanceToObjective < 2.5) {
      Rotation2d.fromDegrees(Util.angleToObjective);
      new Pose2d (Util.deltaOverall_x, Util.deltaOverall_y, m_rotation); {//Pose2d(deltaOverall_x, deltaOverall_y, Rotation2d rotation);
        m_rotation = new Rotation2d(Util.angleToObjective);
        //Use the turret to rotate towards the objective orientation rather than moving forward
      if (Util.angleCorrection < Math.toDegrees(30)){//adjust angleCorrection later on through testing
        RobotContainer.m_driveSubsystem.arcadeDrive(0, 0);
        Util.angleCorrection =  (Constants.turretConstants.kSoftMaxTurretAngle / (360.0 * Constants.turretConstants.kTurretRotationsPerTick));
          //
          //limelightTurretVisionCommand
      }
      }  
    }
    //MAYBE, remember to make sure to stop when another robot is in the way

  }

  
  /**
   * Constructs a pose with the specified translation and rotation.
   *
   * @param translation The translational component of the pose.
   * @param rotation    The rotational component of the pose.
   */
  public void Pose2d(Translation2d translation, Rotation2d rotation) {
    m_translation = translation;
    m_rotation = rotation;
  } 


  // Called once the command ends or is interrupted.
  public void end(final boolean interrupted) {
  }

  // Returns true when the command should end.
  public boolean isFinished() {
    return false;
  }
}
