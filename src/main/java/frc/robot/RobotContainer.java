/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared;
import static frc.robot.Constants.AutoConstants.kMaxSpeedMetersPerSecond;
import static frc.robot.Constants.AutoConstants.kRamseteB;
import static frc.robot.Constants.AutoConstants.kRamseteZeta;
import static frc.robot.Constants.driveConstants.kDriveKinematics;
import java.util.List;

import com.fearxzombie.limelight;
import com.fearxzombie.limelight_mode;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
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
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.driveConstants;
import frc.robot.Constants.pwmConstants;
import frc.robot.commands.turretLimelightCommand;
import frc.robot.commands.turretAutoTargeting;
import frc.robot.commands.turretHomingCommand;
import frc.robot.commands.turretManualMode;
import frc.robot.subsystems.driveSubsystem;
import frc.robot.subsystems.elevatorSubsystem;
import frc.robot.subsystems.indexerSubsystem;
import frc.robot.subsystems.intakeSubsystem;
import frc.robot.subsystems.turretSubsystem;
import frc.robot.subsystems.shooterSubsystem;
import frc.robot.subsystems.controlPanelSubsystem;
import frc.robot.subsystems.blinkinSubsystem;
import frc.robot.commands.elevatorWinchCommand;
import frc.robot.commands.indexerRestageCommand;
import frc.robot.commands.indexerReverseEjectCommand;
import frc.robot.commands.indexerSingleIntakeCommand;
import frc.robot.commands.indexerStageForShootingCommand;
import frc.robot.commands.intakeDeployCommand;
import frc.robot.commands.shootBallsContinuouslyCommand;
import frc.robot.commands.elevatorDeployCommand;

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
  private final indexerSubsystem m_indexer = new indexerSubsystem();
  private final elevatorSubsystem m_elevator = new elevatorSubsystem();
  //private final controlPanelSubsystem m_controlPanelMotors = new controlPanelSubsystem();
  private final intakeSubsystem m_intake = new intakeSubsystem();

  public static XboxController m_driveController = new XboxController(driveConstants.driveController);
  public static XboxController m_operatorController = new XboxController(driveConstants.operatorController);

  public static boolean limelightOnTarget = false;

  public RobotContainer() {
    configureButtonBindings();
    //m_limelight.setLEDMode(limelight_mode.led.on);
    // default command is arcade drive command
    // TODO: un-NERF the drive command.
    m_drive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(() -> m_drive.arcadeDrive(m_driveController.getY(GenericHID.Hand.kLeft), -m_driveController.getX(GenericHID.Hand.kRight)), m_drive));

    //m_turret.setDefaultCommand(new turretLimelightCommand(m_turret, m_shooter, m_limelight));
    m_elevator.setDefaultCommand(new elevatorWinchCommand(m_elevator));
    m_indexer.setDefaultCommand(new indexerSingleIntakeCommand(m_indexer));
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
    // driverYButton.whenPressed(new elevatorDeployCommand(m_elevator));
    // driverXButton.whenPressed(() -> m_elevator.retractElevator(), m_elevator);

      // Left Trigger - climber up (lower robot)
      // Right Trigger - climber down (raise robot)
    
    // Operator Controls
      // Left Joystick - manual turret control
      // Left Trigger - manually move the indexer backwards
      // Right Trigger - manually move the indexer forwards
      // A Button - hold to deploy intake
      opAButton.whileHeld(new intakeDeployCommand(m_intake));
      // B Button - stage balls for shooting
      opBButton.whenPressed(new indexerStageForShootingCommand(m_indexer));
      // X Button - restage balls
      //opXButton.whenPressed(new indexerRestageCommand(m_indexer));
      // Y Button - hold to eject balls out the back of the indexer, restage balls when released
      opYButton.whileHeld(new indexerReverseEjectCommand(m_indexer));
      //opYButton.whenReleased(new indexerRestageCommand(m_indexer));
      // Right Bumper - hold to shoot balls
      opRightBumper.whileHeld(new shootBallsContinuouslyCommand(m_indexer, m_turret, m_shooter, m_limelight, m_drive));
      // D Pad Up - manually increase ball count
      opDPadUp.whenPressed(() -> m_indexer.setBallCount(m_indexer.getBallCount() + 1));
      // D Pad Down - manually decrease ball count
      opDPadDown.whenPressed(() -> m_indexer.setBallCount(m_indexer.getBallCount() - 1));

      
      
    // op Start -> auto targeting
    // op Select -> limelight targeting
    // op A  -> turret home
    // op B  -> manual control with operator controller
    // TODO: do something other than assume power port is directly in front of robot sitting on initiation line
    //Translation2d powerPortLocation = new Translation2d(inches2Meters(120), 0);
    //opStartButton.whenPressed(new turretAutoTargeting(powerPortLocation, m_turret, m_drive, m_limelight));
    opXButton.whenPressed(() -> m_shooter.setShooterRPM(2700));  // 2700 RPM is ideal for 10' (initiation line)
    //opBackButton.whenPressed(new turretLimelightCommand(m_turret, m_shooter, m_limelight));
    //opAButton.whenPressed(new turretHomingCommand(m_turret));
    opBButton.whenPressed(new turretManualMode(m_turret));
  }
  
  public Command getNoAutonomousCommand() {
    return new RunCommand(() -> m_drive.tankDriveVolts(0, 0));
  }
  
  public Command getAutonomousShootCommand() {

    // Spin up flywheel and drive off initiation line
    Command ac = new ParallelRaceGroup(
       getAutonomousCommand(), 
       new InstantCommand(() -> m_shooter.setShooterRPM(m_shooter.getRPMforDistanceFeet(10)), m_shooter));

    // shoot 3 power cells
    ac.andThen(new shootBallsContinuouslyCommand(m_indexer, m_turret, m_shooter, m_limelight, m_drive);

    // TODO: add drive toward nearest powercell and pick up

    return ac;
  }

  public Command getAutonomousCommand() {

    // Drive forward 1.5 meter, 1.5 meter back, and stop
    RamseteCommand ramseteCommand = createTrajectoryCommand(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(new Translation2d(1.5,0.0)),
      new Pose2d(0.0, 0.0, new Rotation2d(0)));

    // TODO: add aim and shoot  
    return ramseteCommand.andThen(() -> m_drive.tankDriveVolts(0, 0));
  }

  public RamseteCommand createTrajectoryCommand(Pose2d startPose, List<Translation2d> translationList, Pose2d endPose) {
    DifferentialDriveVoltageConstraint autoVoltageConstraint;
    TrajectoryConfig config;
  
    // Create a voltage constraint to ensure we don't accelerate too fast
    autoVoltageConstraint = new DifferentialDriveVoltageConstraint(m_drive.getFeedforward(), kDriveKinematics, 6);

    // Create config for trajectory
    config = new TrajectoryConfig(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);

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
  

  
  public double inches2Meters(double i) {
    return i * 0.0254;
  }
}
