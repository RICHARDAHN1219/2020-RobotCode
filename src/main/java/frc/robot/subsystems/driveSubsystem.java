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
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;

public class driveSubsystem extends SubsystemBase {

  private WPI_TalonFX falcon1_leftLead    = new WPI_TalonFX(driveConstants.falcon1_leftLead);
  private WPI_TalonFX falcon2_leftFollow  = new WPI_TalonFX(driveConstants.falcon2_leftFollow);
  private WPI_TalonFX falcon3_rightLead   = new WPI_TalonFX(driveConstants.falcon3_rightLead);
  private WPI_TalonFX falcon4_rightFollow = new WPI_TalonFX(driveConstants.falcon4_rightFollow);

  // OLD Gyro, NAVX:
  //    private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  // New Gyro, pigeon IMU on the CAN bus
  private PigeonIMU m_gyro = new PigeonIMU(driveConstants.pigeonCANid);

  // Note: We do not use SpeedController. We use CAN based Lead/Follow. 

  private final DifferentialDrive m_drive;
  private final SimpleMotorFeedforward  m_feedforward = 
      new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter);

  private PIDController left_PIDController = new PIDController(kPDriveVel, 0.0, kDDriveVel);
  private PIDController right_PIDController =  new PIDController(kPDriveVel, 0.0, kDDriveVel);
 
  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  // http://www.ctr-electronics.com/downloads/pdf/Falcon%20500%20User%20Guide.pdf
  // Peak power: 140A
  // Stall:      257A  (more than the battery can supply)
  // Battery can at best supply around 250A
  private SupplyCurrentLimitConfiguration m_limit =
    // temperary, super low current limit until we sort out speed limits
    new SupplyCurrentLimitConfiguration(true, 20, 10, 0.5);
     // new SupplyCurrentLimitConfiguration(true, 30, 20, 0.5);

  public driveSubsystem() {

    m_gyro.configFactoryDefault();

    m_drive = new DifferentialDrive(falcon1_leftLead, falcon3_rightLead);
    falcon1_leftLead.configFactoryDefault();
    falcon2_leftFollow.configFactoryDefault();
    falcon3_rightLead.configFactoryDefault();
    falcon4_rightFollow.configFactoryDefault();

    // Current limiting
    setCurrentLimit(m_limit);
    
    // set brake mode
    falcon1_leftLead.setNeutralMode(NeutralMode.Brake);
    falcon2_leftFollow.setNeutralMode(NeutralMode.Brake);
    falcon3_rightLead.setNeutralMode(NeutralMode.Brake);
    falcon4_rightFollow.setNeutralMode(NeutralMode.Brake);
    
    // No need to invert Follow Motors
    falcon1_leftLead.setInverted(false);
    falcon3_rightLead.setInverted(true);
    falcon2_leftFollow.setInverted(InvertType.FollowMaster);
    falcon4_rightFollow.setInverted(InvertType.FollowMaster);

    // set Lead/Follow 
    falcon2_leftFollow.follow(falcon1_leftLead);
    falcon4_rightFollow.follow(falcon3_rightLead);

    // NOTE: setSensorPhase() does nothing on TalonFX motors as the encoders 
    // are integrated, and can cannot be out of phase with the motor. 
    falcon1_leftLead.setSensorPhase(true);
    falcon3_rightLead.setSensorPhase(true);

    // default feedback sensor
    falcon1_leftLead.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, driveTimeout);
    falcon3_rightLead.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, driveTimeout);

    m_drive.setRightSideInverted(false);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

=======
  private WPI_TalonFX falcon1 = new WPI_TalonFX(driveConstants.falcon1);
  private WPI_TalonFX falcon2 = new WPI_TalonFX(driveConstants.falcon2);
  private WPI_TalonFX falcon3 = new WPI_TalonFX(driveConstants.falcon3);
  private WPI_TalonFX falcon4 = new WPI_TalonFX(driveConstants.falcon4);
  private PigeonIMU m_pigeon = new PigeonIMU(11);
  private static SpeedController leftSide;
  private static SpeedController rightSide;
  private DifferentialDrive drive;

  public driveSubsystem() {
    falcon1.configSupplyCurrentLimit(Robot.m_currentlimitMain);
    falcon2.configSupplyCurrentLimit(Robot.m_currentlimitMain);
    falcon2.configSupplyCurrentLimit(Robot.m_currentlimitMain);
    falcon4.configSupplyCurrentLimit(Robot.m_currentlimitMain);
    leftSide = new SpeedControllerGroup(falcon1, falcon3);
    rightSide = new SpeedControllerGroup(falcon2, falcon4);
    drive = new DifferentialDrive(leftSide, rightSide);

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

  /**
   * Enable current limiting.
   *
   * @param current limit
   */
  public void setCurrentLimit(SupplyCurrentLimitConfiguration limit) {
    falcon1_leftLead.configSupplyCurrentLimit(limit);
    falcon2_leftFollow.configSupplyCurrentLimit(limit);
    falcon2_leftFollow.configSupplyCurrentLimit(limit);
    falcon4_rightFollow.configSupplyCurrentLimit(limit);
  }

  /**
   * Enable default current limiting for drivetrain.
   */
  public void enableCurrentLimit() {
    setCurrentLimit(m_limit);
  }

  /**
   * Disablecurrent limiting for drivetrain.
   */
  public void disableCurrentLimit() {
    // not completely disabled, 4x80 amps is 240Amps, wich is almost 100% of the battery output
    setCurrentLimit(new SupplyCurrentLimitConfiguration(true, 80, 60, 1));
  }

}
