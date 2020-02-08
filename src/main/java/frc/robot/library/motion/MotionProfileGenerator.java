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
//import frc.robot.Constants.TrajectoryConstants;
//import frc.robot.commands.limelightTurretVisionCommand;
//import edu.wpi.first.wpilibj.controller.RamseteController;




public class MotionProfileGenerator extends CommandBase {
  public static Translation2d m_translation;
  public static Rotation2d m_rotation;
  public static double angleRobotToField;
  public static double angleToObjective;
  public static double angleCorrection;
  public static int pos_x1;
  public static int pos_x2;
  public static int pos_newX;
  public static int pos_y1;
  public static int pos_y2;
  public static int pos_newY;
  public static int delta1_x;
  public static int delta1_y;
  public static int delta2_x;
  public static int delta2_y;
  public static double deltaOverall_x;
  public static double deltaOverall_y;
  public static int pos_y;
  public static int vel_x;
  public static int vel_y;
  public static int acc_x;
  public static int acc_y;
  public static double distanceToObjective;

  /**
   * Uses setPoint Generator to generate an angle we are currently at above the
   * x-axis of the field. We calculate the angle between the horizon and a line
   * determining the distance it takes for the robot to travel from its current
   * point to the new point on the field we want to get to.
   */
  public MotionProfileGenerator() {

  }
  // Called when the command is initially scheduled.
  public void constantsFormulas() {
    delta1_x = Math.abs(pos_x2 - pos_x1);
    delta1_y = Math.abs(pos_y2 - pos_y1);
    delta2_x = Math.abs(pos_newX - pos_x1);
    delta2_y = Math.abs(pos_newY - pos_y1);
    deltaOverall_x = Math.abs(delta1_x - delta2_x);
    deltaOverall_y = Math.abs(delta1_y - delta2_y);
    distanceToObjective = (int) Math.sqrt(((Math.pow((delta1_x - delta2_x), 2.0)) - (Math.pow(((delta1_x - delta2_x)), 2.0))));
    angleRobotToField = (int) Math.atan(delta1_y/delta1_x);
    angleToObjective =(int) Math.atan(delta2_y/delta2_x);
    angleCorrection = angleRobotToField - angleToObjective;
  }


/**Use method createPosePath in order to be able to use the Pose2d to generate a path under certain conditions.
 * Either move the robot forward while adjusting rotation with turret in initial position
 * OR
 * Rotate the robot with the turret in its initial position
 */
 public void createPosePath() {
    if (angleCorrection >= 45 && Robot.turretHome == true && distanceToObjective >= 2.5) {
        //TrajectoryConstants.m_x = deltaOverall_x;
        //TrajectoryConstants.m_y = deltaOverall_y;
      //rotate the robot itself with the characterization
      //proceed to the calculated distanceToObjective
      //this.Pose2d.Pose2d(Translation2d.Translation2d((double)deltaOverall_x, (double)deltaOverall_y), Rotation2d.fromDegrees(angleToObjective));
      Rotation2d.fromDegrees(angleCorrection);
      new Pose2d (deltaOverall_x, deltaOverall_y, m_rotation); {//Pose2d(deltaOverall_x, deltaOverall_y, Rotation2d rotation);
        m_rotation = new Rotation2d(angleToObjective);
        m_translation = new Translation2d(deltaOverall_x, deltaOverall_y); 
      }
      //Use the turret to rotate in order lock onto the vision target from here

      //Translation2d.getDistance(Translation2d other);

      //Pose2d(Translation2d translation, Rotation2d rotation) is the structure
    }
    //If the angle to 
    else if (angleCorrection < 45 && Robot.turretHome == true  && distanceToObjective < 2.5) {
      Rotation2d.fromDegrees(angleToObjective);
      new Pose2d (deltaOverall_x, deltaOverall_y, m_rotation); {//Pose2d(deltaOverall_x, deltaOverall_y, Rotation2d rotation);
        m_rotation = new Rotation2d(angleToObjective);
        //Use the turret to rotate towards the objective orientation rather than moving forward
      }  
    }
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
