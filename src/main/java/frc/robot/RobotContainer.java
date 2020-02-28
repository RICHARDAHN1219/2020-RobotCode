/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.Constants.AutoConstants.kRamseteB;
import static frc.robot.Constants.AutoConstants.kRamseteZeta;
import static frc.robot.Constants.driveConstants.kDriveKinematics;
import java.util.List;
import com.fearxzombie.limelight;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.driveConstants;
import frc.robot.Constants.pwmConstants;
import frc.robot.subsystems.driveSubsystem;
import frc.robot.subsystems.elevatorSubsystem;
import frc.robot.subsystems.indexerSubsystem;
import frc.robot.subsystems.intakeSubsystem;
import frc.robot.subsystems.turretSubsystem;
import frc.robot.subsystems.shooterSubsystem;
import frc.robot.subsystems.blinkinSubsystem;
import frc.robot.commands.*;

public class RobotContainer {

  // Subsystems
  // NOTE: blinkin needs to be first and public static to be accessed by other subsystems
  public final static blinkinSubsystem m_blinkin = new blinkinSubsystem(pwmConstants.blinkin);
  // All other subsystems should be private
  private final driveSubsystem m_drive = new driveSubsystem();
  // public so that it can get the right instance.
  public static final limelight m_limelight = new limelight("limelight-one");
  private final turretSubsystem m_turret = new turretSubsystem();
  public static final shooterSubsystem m_shooter = new shooterSubsystem();
  public static final indexerSubsystem m_indexer = new indexerSubsystem();
  private final elevatorSubsystem m_elevator = new elevatorSubsystem();
  //private final controlPanelSubsystem m_controlPanelMotors = new controlPanelSubsystem();
  private final intakeSubsystem m_intake = new intakeSubsystem();
  public static XboxController m_driveController = new XboxController(driveConstants.driveController);
  public static XboxController m_operatorController = new XboxController(driveConstants.operatorController);
  public static boolean limelightOnTarget = false;

  public RobotContainer() {
    configureButtonBindings();
    m_drive.setDefaultCommand(new driveCommand(m_drive));
    m_elevator.setDefaultCommand(new elevatorWinchCommand(m_elevator));
    m_indexer.setDefaultCommand(new indexerDefaultCommand(m_indexer));
    m_turret.setDefaultCommand(new turretDefaultCommand(m_turret));
  }

  private void configureButtonBindings() {
    
    // Driver Controller Buttons
    final JoystickButton driverAButton = new JoystickButton(m_driveController, Button.kA.value);
    final JoystickButton driverBButton = new JoystickButton(m_driveController, Button.kB.value);
    final JoystickButton driverXButton = new JoystickButton(m_driveController, Button.kX.value);
    final JoystickButton driverYButton = new JoystickButton(m_driveController, Button.kY.value);
    final JoystickButton driverStartButton = new JoystickButton(m_driveController, Button.kStart.value);
    final JoystickButton driverBackButton = new JoystickButton(m_driveController, Button.kBack.value);
    final JoystickButton driverLeftBumper = new JoystickButton(m_driveController, Button.kBumperLeft.value);
    final JoystickButton driverRightBumper = new JoystickButton(m_driveController, Button.kBumperRight.value);
    
    // Operator Controller Buttons
    final JoystickButton opAButton = new JoystickButton(m_operatorController, Button.kA.value);
    final JoystickButton opBButton = new JoystickButton(m_operatorController, Button.kB.value);
    final JoystickButton opXButton = new JoystickButton(m_operatorController, Button.kX.value);
    final JoystickButton opYButton = new JoystickButton(m_operatorController, Button.kY.value);
    final JoystickButton opStartButton = new JoystickButton(m_operatorController, Button.kStart.value);
    final JoystickButton opBackButton = new JoystickButton(m_operatorController, Button.kBack.value);
    final JoystickButton opLeftBumper = new JoystickButton(m_operatorController, Button.kBumperLeft.value);
    final JoystickButton opRightBumper = new JoystickButton(m_operatorController, Button.kBumperRight.value);
    final POVButton opDPadUp = new POVButton(m_operatorController, 0);
    final POVButton opDPadDown = new POVButton(m_operatorController, 180);
    
    // Driver Controls
      // Y Button to deploy the elevator
      // X Button to retract the elevator
      // Left Trigger - climber down (raise robot)
      // Right Trigger - climber up (lower robot)
      // Right Bumper - invert drive controls
      driverYButton.whenPressed(() -> m_elevator.deployElevator());
      driverXButton.whenPressed(() -> m_elevator.retractElevator());
      driverRightBumper.whenPressed(new driveInvertCommand(m_drive));
    
    // Operator Controls
      // Left Joystick - manual turret control
      // Left Trigger - manually move the indexer backwards
      // Right Trigger - manually move the indexer forwards
      // A Button - hold to deploy intake
      // B Button - stage balls for shooting
      // X Button - restage balls
      // Y Button - hold to eject balls out the back of the indexer
      // Right Bumper - shoot with hood up
      // Left Bumper - shoot with hood down
      // D Pad Up - manually increase ball count
      // D Pad Down - manually decrease ball count
      // Start Button - zero the turret
      // Back Button - spool up the shooter
      opAButton.whileHeld(new intakeDeployCommand(m_intake));
      opBButton.whenPressed(new indexerStageForShootingCommand(m_indexer));
      opXButton.whenPressed(new indexerRestageCommand(m_indexer));
      opYButton.whileHeld(new indexerReverseEjectCommand(m_indexer));
      opRightBumper.whileHeld(new hoodUpAutoShootCommand(m_indexer, m_turret, m_shooter, m_limelight));
      opLeftBumper.whileHeld(new hoodDownAutoShootCommand(m_indexer, m_turret, m_shooter, m_limelight));
      opDPadUp.whenPressed(() -> m_indexer.setBallCount(m_indexer.getBallCount() + 1));
      opDPadDown.whenPressed(() -> m_indexer.setBallCount(m_indexer.getBallCount() - 1));
      opBackButton.toggleWhenPressed(new shooterSpoolCommand(m_shooter));
  }
  
  /**
   * Do nothing during auton
   * 
   * @return Auton do nothing command
   */
  public Command noAutonomous() {
    return new RunCommand(() -> m_drive.tankDriveVolts(0, 0));
  }

  public Command straightOn3Ball() {
    RamseteCommand moveBack1 = createTrajectoryCommand(new Pose2d(0, 0, new Rotation2d(0)), List.of(new Translation2d(-0.5, 0)), new Pose2d(-1, 0, new Rotation2d(0)), true, 2.5, 0.75);
    
    return 
    new InstantCommand(() -> m_shooter.setShooterPID(0.0005, 0.00000025, 0, 0.00022, 250), m_shooter).
    andThen(moveBack1.alongWith(new InstantCommand(() -> m_shooter.setShooterRPM(3550), m_shooter))).
    andThen(new hoodUpAutoShootCommand(m_indexer, m_turret, m_shooter, m_limelight));
  }

  public Command rightSide3Ball() {
    RamseteCommand moveBack1 = createTrajectoryCommand(new Pose2d(0, 0, new Rotation2d(0)), List.of(new Translation2d(-0.1, 0)), new Pose2d(-1, 0, new Rotation2d(0)), true, 2.5, 0.75);

    return 
    new InstantCommand(() -> m_shooter.setShooterPID(0.0005, 0.00000025, 0, 0.00022, 250), m_shooter).
    andThen(moveBack1.alongWith(new InstantCommand(() -> m_shooter.setShooterRPM(3550), m_shooter), new InstantCommand(() -> m_turret.setAngleDegrees(-5), m_turret))).
    andThen(new hoodUpAutoShootCommand(m_indexer, m_turret, m_shooter, m_limelight));
  }

  public Command rightSide4Ball() {
    RamseteCommand moveBack1 = createTrajectoryCommand(new Pose2d(0, 0, new Rotation2d(0)), List.of(new Translation2d(-1.15, 0)), new Pose2d(-2.54, 0, new Rotation2d(0)), true, 2.5, 1);
    RamseteCommand moveForward2 = createTrajectoryCommand(new Pose2d(-2.54, 0, new Rotation2d(0)), List.of(new Translation2d(-1.15, 0)), new Pose2d(0, 0, new Rotation2d(0)), false, 3.5, 1.5);
    
    return 
    new InstantCommand(() -> m_shooter.setShooterPID(0.0005, 0.00000025, 0, 0.00022, 250), m_shooter).
    andThen(moveBack1.deadlineWith(new intakeDeployCommand(m_intake), new indexerDefaultCommand(m_indexer).perpetually())).
    andThen(moveForward2.alongWith(new InstantCommand(() -> m_shooter.setShooterRPM(3550), m_shooter), new InstantCommand(() -> m_turret.setAngleDegrees(-3), m_turret)).
    andThen(new hoodUpAutoShootCommand(m_indexer, m_turret, m_shooter, m_limelight)));
  }

  public Command rightSide5Ball() {
    RamseteCommand moveBack1 = createTrajectoryCommand(new Pose2d(0, 0, new Rotation2d(0)), List.of(new Translation2d(-1.15, 0)), new Pose2d(-2.54, 0, new Rotation2d(0)), true, 2, 1);
    RamseteCommand moveBack2 = createTrajectoryCommand(new Pose2d(-2.54, 0, new Rotation2d(0)), List.of(new Translation2d(-2.6, 0)), new Pose2d(-3.5, 0, new Rotation2d(0)), true, 2.5, 1);
    RamseteCommand moveForward3 = createTrajectoryCommand(new Pose2d(-2.85, 0, new Rotation2d(0)), List.of(new Translation2d(-2, 0)), new Pose2d(-1.5, 0, new Rotation2d(0)), false, 2.5, 1);
    
    return 
    new InstantCommand(() -> m_shooter.setShooterPID(0.0005, 0.00000025, 0, 0.00022, 250), m_shooter).
    andThen(moveBack1.deadlineWith(new intakeDeployCommand(m_intake), new indexerDefaultCommand(m_indexer).perpetually())).
    andThen(moveBack2.deadlineWith(new intakeDeployCommand(m_intake), new indexerDefaultCommand(m_indexer).perpetually())).
    andThen(new WaitCommand(2).deadlineWith(new intakeDeployCommand(m_intake), new indexerDefaultCommand(m_indexer).perpetually())).
    andThen(moveForward3.alongWith(new InstantCommand(() -> m_shooter.setShooterRPM(3550), m_shooter), new InstantCommand(() -> m_turret.setAngleDegrees(-3), m_turret), new indexerStageForShootingCommand(m_indexer))).
    andThen(new hoodUpAutoShootCommand(m_indexer, m_turret, m_shooter, m_limelight));
  }

  public Command rightSide6Ball() {
    RamseteCommand moveBack1 = createTrajectoryCommand(new Pose2d(0, 0, new Rotation2d(0)), List.of(new Translation2d(-1.15, 0)), new Pose2d(-2.49, 0, new Rotation2d(0)), true, 3, 2);
    RamseteCommand moveBack2 = createTrajectoryCommand(new Pose2d(-2.49, 0, new Rotation2d(0)), List.of(new Translation2d(-2.6, 0)), new Pose2d(-3.35, 0, new Rotation2d(0)), true, 3, 1.5);
    RamseteCommand moveBack3 = createTrajectoryCommand(new Pose2d(-3.35, 0, new Rotation2d(0)), List.of(new Translation2d(-3.6, 0)), new Pose2d(-4.35, 0, new Rotation2d(0)), true, 3, 1.5);
    RamseteCommand moveForward4 = createTrajectoryCommand(new Pose2d(-4.35, 0, new Rotation2d(0)), List.of(new Translation2d(-4, 0)), new Pose2d(-1.5, 0, new Rotation2d(0)), false, 3.75, 2.75);

    return 
    new InstantCommand(() -> m_shooter.setShooterPID(0.0005, 0.00000025, 0, 0.00022, 250), m_shooter).alongWith(new InstantCommand(() -> m_indexer.setBallCount(3))).
    andThen(new InstantCommand(() -> m_turret.setAngleDegrees(-3), m_turret)).
    andThen(new WaitUntilCommand(() -> m_indexer.getBallCount() == 0).deadlineWith(new hoodUpAutoShootCommand(m_indexer, m_turret, m_shooter, m_limelight))).
    andThen(moveBack1.deadlineWith(new intakeDeployCommand(m_intake), new indexerDefaultCommand(m_indexer).perpetually())).
    andThen(moveBack2.deadlineWith(new intakeDeployCommand(m_intake), new indexerDefaultCommand(m_indexer).perpetually())).
    andThen(new WaitCommand(0.5).deadlineWith(new intakeDeployCommand(m_intake), new indexerDefaultCommand(m_indexer).perpetually())).
    andThen(moveBack3.deadlineWith(new intakeDeployCommand(m_intake), new indexerDefaultCommand(m_indexer).perpetually()).
    andThen(new WaitCommand(0.75).deadlineWith(new intakeDeployCommand(m_intake), new indexerDefaultCommand(m_indexer).perpetually())).
    andThen(moveForward4.deadlineWith(new InstantCommand(() -> m_shooter.setShooterRPM(3550), m_shooter), new InstantCommand(() -> m_turret.setAngleDegrees(-3), m_turret), new indexerStageForShootingCommand(m_indexer)))).
    andThen(new hoodUpAutoShootCommand(m_indexer, m_turret, m_shooter, m_limelight));
  }

  public Command rightSide6BallTest() {
    RamseteCommand moveBack1 = createTrajectoryCommand(new Pose2d(0, 0, new Rotation2d(0)), List.of(new Translation2d(-1.15, 0)), new Pose2d(-2.3, 0, new Rotation2d(0)), true, 3, 2);
    RamseteCommand moveBack2 = createTrajectoryCommand(new Pose2d(-2.3, 0, new Rotation2d(0)), List.of(new Translation2d(-2.6, 0)), new Pose2d(-4.35, 0, new Rotation2d(0)), true, 1, 0.5);
    RamseteCommand moveForward3 = createTrajectoryCommand(new Pose2d(-4.35, 0, new Rotation2d(0)), List.of(new Translation2d(-4, 0)), new Pose2d(-1.5, 0, new Rotation2d(0)), false, 3.75, 2.75);

    // To try and save time, this version continues through the 3 balls slowly rather than stopping at each one
    return 
    new InstantCommand(() -> m_shooter.setShooterPID(0.0005, 0.00000025, 0, 0.00022, 250), m_shooter).alongWith(new InstantCommand(() -> m_indexer.setBallCount(3))).
    andThen(new InstantCommand(() -> m_turret.setAngleDegrees(-3), m_turret)).
    andThen(new WaitUntilCommand(() -> m_indexer.getBallCount() == 0).deadlineWith(new hoodUpAutoShootCommand(m_indexer, m_turret, m_shooter, m_limelight))).
    andThen(moveBack1.deadlineWith(new intakeDeployCommand(m_intake), new indexerDefaultCommand(m_indexer).perpetually())).
    andThen(moveBack2.deadlineWith(new intakeDeployCommand(m_intake), new indexerDefaultCommand(m_indexer).perpetually())).
    andThen(new WaitCommand(0.5).deadlineWith(new intakeDeployCommand(m_intake), new indexerDefaultCommand(m_indexer).perpetually())).
    andThen(moveForward3.deadlineWith(new InstantCommand(() -> m_shooter.setShooterRPM(3550), m_shooter), new InstantCommand(() -> m_turret.setAngleDegrees(-3), m_turret), new indexerStageForShootingCommand(m_indexer))).
    andThen(new hoodUpAutoShootCommand(m_indexer, m_turret, m_shooter, m_limelight));
  }

  public Command middle3Ball() {
    RamseteCommand moveBack1 = createTrajectoryCommand(new Pose2d(0, 0, new Rotation2d(0)), List.of(new Translation2d(-0.1, 0)), new Pose2d(-1, 0, new Rotation2d(0)), true, 2.5, 0.75);
    
    return 
    new InstantCommand(() -> m_shooter.setShooterPID(0.0005, 0.00000025, 0, 0.00022, 250), m_shooter).
    andThen(moveBack1.alongWith(new InstantCommand(() -> m_shooter.setShooterRPM(3550), m_shooter), new InstantCommand(() -> m_turret.setAngleDegrees(5), m_turret))).
    andThen(new hoodUpAutoShootCommand(m_indexer, m_turret, m_shooter, m_limelight));
  }

  public Command middle4BallTest() {
    RamseteCommand moveBack1 = createTrajectoryCommand(new Pose2d(0, 0, new Rotation2d(0)), List.of(new Translation2d(-0.75, -0.25)), new Pose2d(-2.54, -0.5, new Rotation2d(0)), true, 3.5, 1.5);
    RamseteCommand moveForward2 = createTrajectoryCommand(new Pose2d(-2.54, -0.5, new Rotation2d(0)), List.of(new Translation2d(-0.75, -0.25)), new Pose2d(0, 0, new Rotation2d(0)), true, 3.5, 1.5);
    
    return
    new InstantCommand(() -> m_shooter.setShooterPID(0.0005, 0.00000025, 0, 0.00022, 250), m_shooter).
    andThen(moveBack1.deadlineWith(new intakeDeployCommand(m_intake), new indexerDefaultCommand(m_indexer).perpetually())).
    andThen(moveForward2.alongWith(new InstantCommand(() -> m_shooter.setShooterRPM(3550), m_shooter), new InstantCommand(() -> m_turret.setAngleDegrees(3)))).
    andThen(new hoodUpAutoShootCommand(m_indexer, m_turret, m_shooter, m_limelight));
  }

  /**
   * createTrajectoryCommand - given a start pose, some intermediate points, and a finish pose, create
   *     a Ramsete Command to execute the path follow.
   * 
   * @param startPose
   * @param translationList
   * @param endPose
   * @param isReversed
   * @param maxSpeedMetersPerSecond
   * @param maxAccelerationMetersPerSecondSquared
   * @return Ramsete Path Follow Command, intake side of robot is isReversed = true and negative values
   */
  public RamseteCommand createTrajectoryCommand(Pose2d startPose, List<Translation2d> translationList, Pose2d endPose, boolean isReversed, double maxSpeedMetersPerSecond, double maxAccelerationMetersPerSecondSquared) {
    DifferentialDriveVoltageConstraint autoVoltageConstraint;
    TrajectoryConfig config;
  
    // Create a voltage constraint to ensure we don't accelerate too fast
    autoVoltageConstraint = new DifferentialDriveVoltageConstraint(m_drive.getFeedforward(), kDriveKinematics, 6);

    // Create config for trajectory
    config = new TrajectoryConfig(maxSpeedMetersPerSecond, maxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint)
        .setReversed(isReversed);

    var initalTime = System.nanoTime();

    // trajectory to follow. All units in meters.
   var trajectory = TrajectoryGenerator.generateTrajectory(
        startPose,
        translationList,
        endPose,
        config);

    RamseteCommand ramseteCommand =
        new RamseteCommand(trajectory, 
            m_drive::getPose,
            new RamseteController(kRamseteB, kRamseteZeta),
            m_drive.getFeedforward(),
            kDriveKinematics,
            m_drive::getWheelSpeeds,
            m_drive.getLeftPidController(),
            m_drive.getRightPidController(),
            m_drive::tankDriveVolts,
            m_drive);

    var dt = (System.nanoTime() - initalTime) / 1E6;
    System.out.println("RamseteCommand generation time: " + dt + "ms");

    // Run path following command, then stop at the end.
    return ramseteCommand;
  }
  

  // TODO: this should be in Math Utils or a new Units.java under Utils
  public double inches2Meters(double i) {
    return i * 0.0254;
  }
}
