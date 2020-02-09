/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.driveConstants;
import frc.robot.commands.driveCommand;
import frc.robot.commands.limelightTurretVisionCommand;
import frc.robot.commands.manualMode;
import frc.robot.commands.shooterCommand;
import frc.robot.commands.turretHomingCommand;
import frc.robot.subsystems.driveSubsystem;
import frc.robot.subsystems.elevatorSubsystem;
import frc.robot.subsystems.indexerSubsystem;
import frc.robot.subsystems.turretSubsystem;
import frc.robot.subsystems.shooterSubsystem;
//import frc.robot.subsystems.controlPanelSubsystem;

public class RobotContainer {
  @SuppressWarnings("unused")
  // Subsystems
  private final driveSubsystem m_driveSubsystem = new driveSubsystem();
  private final turretSubsystem m_turretSubsystem = new turretSubsystem();
  private final shooterSubsystem m_shooter = new shooterSubsystem();
  private final indexerSubsystem m_indexer = new indexerSubsystem();
  @SuppressWarnings("unused")
  private final elevatorSubsystem m_elevatorSubsystem = new elevatorSubsystem();
  //private final controlPanelSubsystem m_controlPanelMotors = new controlPanelSubsystem();

  // Commands
  //public static final shooterCommand m_shooterCommand = new shooterCommand(m_shooter, m_indexer);
  //private final limelightTurretVisionCommand m_turretVisionCommand = new limelightTurretVisionCommand(m_turretSubsystem);
  //private final driveCommand m_driveCommand = new driveCommand(m_driveSubsystem);
  
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public static XboxController m_driveController = new XboxController(driveConstants.driveController);
  public static XboxController m_operatorController = new XboxController(driveConstants.operatorController);
  public RobotContainer() {
    configureButtonBindings();
    m_driveSubsystem.setDefaultCommand(new driveCommand(m_driveSubsystem));
    m_turretSubsystem.setDefaultCommand(new limelightTurretVisionCommand(m_turretSubsystem));
  }

  private void configureButtonBindings() {
    final JoystickButton abutton = new JoystickButton(m_driveController, Button.kA.value);
    final JoystickButton bbutton = new JoystickButton(m_driveController, Button.kBumperRight.value);
    final JoystickButton xbutton = new JoystickButton(m_driveController, Button.kX.value);
    final JoystickButton opBbutton = new JoystickButton(m_operatorController, Button.kBumperRight.value);
    final JoystickButton ybutton = new JoystickButton(m_driveController, Button.kY.value);
    bbutton.toggleWhenPressed(new shooterCommand(m_shooter, m_indexer));
    final JoystickButton opAbutton = new JoystickButton(m_driveController, Button.kA.value);
    opAbutton.whenPressed(new manualMode());
    opBbutton.whenPressed(new turretHomingCommand());
    //ybutton.toggleWhenPressed(new indexStage1Command(m_indexer));
    //ybutton.whenPressed(() -> m_controlPanelMotors.setPosition(0), m_controlPanelMotors);
    //xbutton.whenPressed(() -> m_controlPanelMotors.setPosition(1 * 4096), m_controlPanelMotors);
    abutton.whenPressed(() -> m_shooter.setShooterRPM(2000));
    xbutton.whenPressed(() -> m_shooter.setShooterRPM(2500));
    ybutton.whenPressed(() -> m_shooter.setShooterRPM(3000));
  }
  /*
  public Command getAutonomousCommand() {
    An ExampleCommand will run in autonomous
    limelightVisionCommand m_autoCommand = new limelightVisionCommand();
    return m_autoCommand;
  }
  */
}
