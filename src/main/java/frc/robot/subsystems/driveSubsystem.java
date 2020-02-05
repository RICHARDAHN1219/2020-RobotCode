/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.driveConstants.kaVoltSecondsSquaredPerMeter;
import static frc.robot.Constants.driveConstants.ksVolts;
import static frc.robot.Constants.driveConstants.kvVoltSecondsPerMeter;
import static frc.robot.Constants.driveConstants.kPDriveVel;
import static frc.robot.Constants.driveConstants.kDDriveVel;
import static frc.robot.Constants.driveConstants.kEncoderCPR;
import static frc.robot.Constants.driveConstants.kDistancePerWheelRevolutionMeters;
import static frc.robot.Constants.driveConstants.kGearReduction;
import static frc.robot.Constants.driveConstants.kGyroReversed;
import static frc.robot.Constants.driveConstants.driveTimeout;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.driveConstants;

public class driveSubsystem extends SubsystemBase {

  private WPI_TalonFX falcon1_leftLead    = new WPI_TalonFX(driveConstants.falcon1_leftLead);
  private WPI_TalonFX falcon2_leftFollow  = new WPI_TalonFX(driveConstants.falcon2_leftFollow);
  private WPI_TalonFX falcon3_rightLead   = new WPI_TalonFX(driveConstants.falcon3_rightLead);
  private WPI_TalonFX falcon4_rightFollow = new WPI_TalonFX(driveConstants.falcon4_rightFollow);
  //private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);
  private PigeonIMU m_gyro = new PigeonIMU(driveConstants.pigeonCANid);

  //private static SpeedController m_leftMotors;
  //private static SpeedController m_rightMotors;

  private final DifferentialDrive m_drive;
  private final TalonFXSensorCollection m_leftEncoder;
  private final TalonFXSensorCollection m_rightEncoder;
  private final SimpleMotorFeedforward  m_feedforward = 
      new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter);

  private PIDController left_PIDController = new PIDController(kPDriveVel, 0.0, kDDriveVel);
  private PIDController right_PIDController =  new PIDController(kPDriveVel, 0.0, kDDriveVel);

  // The gyro sensor
  // private final Gyro m_gyro = new ADXRS450_Gyro();
 
  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  public driveSubsystem() {

    falcon1_leftLead.configFactoryDefault();
    falcon2_leftFollow.configFactoryDefault();
    falcon3_rightLead.configFactoryDefault();
    falcon4_rightFollow.configFactoryDefault();

    // Current limiting
    falcon1_leftLead.configSupplyCurrentLimit(Robot.m_currentlimitMain);
    falcon2_leftFollow.configSupplyCurrentLimit(Robot.m_currentlimitMain);
    falcon2_leftFollow.configSupplyCurrentLimit(Robot.m_currentlimitMain);
    falcon4_rightFollow.configSupplyCurrentLimit(Robot.m_currentlimitMain);

    // set brake mode
    falcon1_leftLead.setNeutralMode(NeutralMode.Brake);
    falcon2_leftFollow.setNeutralMode(NeutralMode.Brake);
    falcon3_rightLead.setNeutralMode(NeutralMode.Brake);
    falcon4_rightFollow.setNeutralMode(NeutralMode.Brake);
    
    // TODO: do we need to invert both right motors?
    falcon3_rightLead.setInverted(true);
    falcon4_rightFollow.setInverted(true);
    // falcon3_rightLead.setSensorPhase(false);

    // default feed
    falcon1_leftLead.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, driveTimeout);
    falcon3_rightLead.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, driveTimeout);


    // set Lead/Follow 
    falcon2_leftFollow.follow(falcon1_leftLead);
    falcon4_rightFollow.follow(falcon3_rightLead);
 
    m_leftEncoder = falcon1_leftLead.getSensorCollection();
    m_rightEncoder = falcon3_rightLead.getSensorCollection();

    m_drive = new DifferentialDrive(falcon1_leftLead, falcon3_rightLead);
    m_drive.setRightSideInverted(false);

    m_gyro.configFactoryDefault();

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

  }
  
  @Override
  public void periodic() {
        // Note: periodic() is run by the scheduler, always. No matter what.
    // Update the odometry in the periodic block
    double leftDist = getPosition(m_leftEncoder); 
    double rightDist = getPosition(m_rightEncoder);
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), leftDist, rightDist);

    // log drive train and data to Smartdashboard
    /* Display 9-axis Heading (requires magnetometer calibration to be useful) */
    SmartDashboard.putNumber("IMU_FusedHeading", m_gyro.getFusedHeading());
    // NOTE: call getFusedHeading(FusionStatus) to detect gyro errors

    // report the wheel speed, position, and pose
    SmartDashboard.putNumber("left_wheel_Velocity",  getVelocity(m_leftEncoder));
    SmartDashboard.putNumber("right_wheel_Velocity", getVelocity(m_rightEncoder));
    SmartDashboard.putNumber("left_wheel_Distance", leftDist); // m_leftEncoder.getPosition());
    SmartDashboard.putNumber("right_wheel_Distance", rightDist); // m_rightEncoder.getPosition());

    Pose2d currentPose = m_odometry.getPoseMeters();
    SmartDashboard.putNumber("pose_x",currentPose.getTranslation().getX());
    SmartDashboard.putNumber("pose_y",currentPose.getTranslation().getY());
    SmartDashboard.putNumber("pose_theta", currentPose.getRotation().getDegrees());
  }
  
  /**
   * Returns the distance in Meteres wheel has travelled
   *
   * @return distance in meters
   */
   double getPosition(TalonFXSensorCollection encoder) {
     // Native units are encoder ticks (2048 ticks per revolution)
    return encoder.getIntegratedSensorPosition() * kDistancePerWheelRevolutionMeters * kGearReduction / kEncoderCPR;
   }

  /**
   * Returns the velocity of a given wheel in meters per second
   *
   * @return velocity in meters/second
   */
  double getVelocity(TalonFXSensorCollection encoder) {
    // Native units are encoder ticks per 100ms
    return encoder.getIntegratedSensorVelocity() * kDistancePerWheelRevolutionMeters * kGearReduction / (kEncoderCPR * 10.0);
   }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the Feedforward settings for the drivetrain.
   * 
   * @return Feedforward
   */
  public SimpleMotorFeedforward getFeedforward() {
    return m_feedforward;
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        getVelocity(m_leftEncoder),
        getVelocity(m_rightEncoder));
  }

  /**
   * Returns the left PIDController object
   *
   * @return PIDController
   */
  public PIDController getLeftPidController() {
    return left_PIDController;
  }

  /**
   * Returns the right PIDController object
   *
   * @return PIDController
   */
  public PIDController getRightPidController() {
    return right_PIDController;
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    falcon1_leftLead.setVoltage(leftVolts);
    falcon3_rightLead.setVoltage(rightVolts);
    m_drive.feed();
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    m_leftEncoder.setIntegratedSensorPosition(0.0, driveTimeout);
    m_rightEncoder.setIntegratedSensorPosition(0.0, driveTimeout);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    // Was Encoder.getDistance()
    return ((m_leftEncoder.getIntegratedSensorPosition() + 
             m_rightEncoder.getIntegratedSensorPosition()) / 2.0);
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public TalonFXSensorCollection getLeftEncoder() {
    return m_leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public TalonFXSensorCollection getRightEncoder() {
    return m_rightEncoder;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    m_gyro.setFusedHeading(0.0);
    m_gyro.setAccumZAngle(0.0);
    // was m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    // m_gyro.getFusedHeading() returns degrees
    return Math.IEEEremainder(m_gyro.getFusedHeading(), 360) * (driveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    double [] xyz_dps = new double [3];
    // getRawGyro returns in degrees/second
    m_gyro.getRawGyro(xyz_dps);
    return xyz_dps[2] * (kGyroReversed ? -1.0 : 1.0);
  }
}
