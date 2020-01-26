/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.driveSubsystem;
import frc.robot.subsystems.elevatorSubsystem;
import frc.robot.subsystems.shooterSubsystem;
import frc.robot.subsystems.turretSubsystem;
import com.ctre.phoenix.motorcontrol.*;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public static boolean manualMode = false;
  public static boolean turretHome = false;
  @SuppressWarnings("unused")
  private RobotContainer m_robotContainer;
  boolean sensor1Last = true;
  boolean sensor2Last = true;
  boolean sensor3Last = true;
  int stateChangeCount = 0;
  SupplyCurrentLimitConfiguration m_currentlimitMain = new SupplyCurrentLimitConfiguration(true, 35, 1, 1);
  SupplyCurrentLimitConfiguration m_currentlimitSecondary = new SupplyCurrentLimitConfiguration(true, 25, 1, 1);

  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    turretSubsystem.turretDrive.setSelectedSensorPosition(0, 0, 10); 
    turretSubsystem.turretDrive.getSensorCollection().setQuadraturePosition(0, 10);
    turretSubsystem.turretDrive.configContinuousCurrentLimit(25);
    driveSubsystem.falcon1.configSupplyCurrentLimit(m_currentlimitMain);
    driveSubsystem.falcon2.configSupplyCurrentLimit(m_currentlimitMain);
    driveSubsystem.falcon2.configSupplyCurrentLimit(m_currentlimitMain);
    driveSubsystem.falcon4.configSupplyCurrentLimit(m_currentlimitMain);
    shooterSubsystem.shooter1.configSupplyCurrentLimit(m_currentlimitSecondary);
    shooterSubsystem.shooter2.configSupplyCurrentLimit(m_currentlimitSecondary);
    IndexerSubsystem.indexStage1_1.configSupplyCurrentLimit(m_currentlimitSecondary);
    IndexerSubsystem.indexStage1_2.configSupplyCurrentLimit(m_currentlimitSecondary);
    IndexerSubsystem.indexLoad.configSupplyCurrentLimit(m_currentlimitSecondary);
    elevatorSubsystem.elevatorWinch.configSupplyCurrentLimit(m_currentlimitMain);
    elevatorSubsystem.elevatorWinch.setNeutralMode(NeutralMode.Coast);
    
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    boolean sensor1 = IndexerSubsystem.Sensor1.get();
    boolean sensor2 = IndexerSubsystem.Sensor2.get();
    boolean sensor3 = IndexerSubsystem.Sensor3.get();
    boolean turretLimit1 = turretSubsystem.limit1.get();
    boolean turretLimit2 = turretSubsystem.limit2.get();
    IndexerSubsystem.indexStage1_2.follow(IndexerSubsystem.indexStage1_1);
    IndexerSubsystem.indexStage1_1.setInverted(true);
    SmartDashboard.putNumber("ball count", IndexerSubsystem.ballCount);
    SmartDashboard.putNumber("state change count", stateChangeCount);
    if (turretLimit1 == true) {
      turretSubsystem.turretDrive.set(ControlMode.PercentOutput, -.5);
      DriverStation.reportError("Limit Reached on turret, going back to safe position.", false);
    }
    if (turretLimit2 == true) {
      turretSubsystem.turretDrive.set(ControlMode.PercentOutput, .5);
      DriverStation.reportError("Limit Reached on turret, going back to safe position.", f);
    }
       
    
    if (sensor1 == false) {
      IndexerSubsystem.indexStage1_1.set(ControlMode.PercentOutput, 0.75);
      IndexerSubsystem.indexLoad.set(ControlMode.PercentOutput, 0.75);
    } 
    
    else if (sensor2 == false) {
      IndexerSubsystem.indexStage1_1.set(ControlMode.PercentOutput, 0);
      IndexerSubsystem.indexLoad.set(ControlMode.PercentOutput, 0);
    }

    if (sensor1 != sensor1Last && sensor1 == false) {
      IndexerSubsystem.ballCount += 1;
      sensor1Last = sensor1;
    }

    if (sensor1 != sensor1Last && sensor1 == true) {
      sensor1Last = sensor1;
    }

    if (sensor3 != sensor3Last && sensor3 == false) {
      IndexerSubsystem.ballCount -= 1;
      sensor3Last = sensor3;
    }

    if (sensor3 != sensor3Last && sensor3 == true) {
      sensor3Last = sensor3;
    }

    if ((IndexerSubsystem.ballCount >= 1) && sensor1 == true && sensor2 == true) {
      IndexerSubsystem.indexStage1_1.set(ControlMode.PercentOutput, 0.75);
    }

    if (sensor2 != sensor2Last && sensor2 == false) {
      stateChangeCount += 1;
      sensor2Last = sensor2;
    }

    if (sensor2 != sensor2Last && sensor2 == true) {
      stateChangeCount += 1;
      sensor2Last = sensor2;
    }
    
    if (IndexerSubsystem.ballCount == 1 && stateChangeCount != 1) {
      IndexerSubsystem.indexStage1_1.set(ControlMode.PercentOutput, 0.75);
      IndexerSubsystem.indexLoad.set(ControlMode.PercentOutput, 0.75);
    }
    
    if (IndexerSubsystem.ballCount == 2 && stateChangeCount != 3) {
      IndexerSubsystem.indexStage1_1.set(ControlMode.PercentOutput, 0.75);
      IndexerSubsystem.indexLoad.set(ControlMode.PercentOutput, 0.75);
      if (IndexerSubsystem.ballCount == 2 && stateChangeCount == 3) {
        IndexerSubsystem.indexStage1_1.set(ControlMode.PercentOutput, 0);
        IndexerSubsystem.indexLoad.set(ControlMode.PercentOutput, 0);
      }
    }
    /*else {
      IndexerSubsystem.indexStage1_1.set(ControlMode.PercentOutput, 0);
      IndexerSubsystem.indexLoad.set(ControlMode.PercentOutput, 0);
    }*/
    
    if (IndexerSubsystem.ballCount == 3 && stateChangeCount != 5) {
      IndexerSubsystem.indexStage1_1.set(ControlMode.PercentOutput, 0.75);
      IndexerSubsystem.indexLoad.set(ControlMode.PercentOutput, 0.75);
      if (IndexerSubsystem.ballCount == 3 && stateChangeCount == 5) {
        IndexerSubsystem.indexStage1_1.set(ControlMode.PercentOutput, 0);
        IndexerSubsystem.indexLoad.set(ControlMode.PercentOutput, 0);
      }
    }
/*    else {
      IndexerSubsystem.indexStage1_1.set(ControlMode.PercentOutput, 0);
      IndexerSubsystem.indexLoad.set(ControlMode.PercentOutput, 0);
    }*/

    if (IndexerSubsystem.ballCount == 4 && stateChangeCount != 7) {
      IndexerSubsystem.indexStage1_1.set(ControlMode.PercentOutput, 0.75);
      IndexerSubsystem.indexLoad.set(ControlMode.PercentOutput, 0.75);
      if (IndexerSubsystem.ballCount == 4 && stateChangeCount == 7) {
        IndexerSubsystem.indexStage1_1.set(ControlMode.PercentOutput, 0);
        IndexerSubsystem.indexLoad.set(ControlMode.PercentOutput, 0);
      }
    }
    /*else {
      IndexerSubsystem.indexStage1_1.set(ControlMode.PercentOutput, 0);
      IndexerSubsystem.indexLoad.set(ControlMode.PercentOutput, 0);
    }*/

    if (IndexerSubsystem.ballCount == 5 && stateChangeCount != 9) {
      IndexerSubsystem.indexStage1_1.set(ControlMode.PercentOutput, 0.75);
      IndexerSubsystem.indexLoad.set(ControlMode.PercentOutput, 0.75);
      if (IndexerSubsystem.ballCount == 5 && stateChangeCount == 9) {
        IndexerSubsystem.indexStage1_1.set(ControlMode.PercentOutput, 0);
        IndexerSubsystem.indexLoad.set(ControlMode.PercentOutput, 0);
      }
    }
    /*else {
      IndexerSubsystem.indexStage1_1.set(ControlMode.PercentOutput, 0);
      IndexerSubsystem.indexLoad.set(ControlMode.PercentOutput, 0);
    }*/
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    //m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
