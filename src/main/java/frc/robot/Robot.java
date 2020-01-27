package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.indexerSubsystem;
import frc.robot.subsystems.elevatorSubsystem;
import frc.robot.subsystems.turretSubsystem;
import com.ctre.phoenix.motorcontrol.*;

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
  int ballCount = 0;
  public static SupplyCurrentLimitConfiguration m_currentlimitMain = new SupplyCurrentLimitConfiguration(true, 35, 1, 1);
  public static SupplyCurrentLimitConfiguration m_currentlimitSecondary = new SupplyCurrentLimitConfiguration(true, 25, 1, 1);

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    turretSubsystem.turretDrive.setSelectedSensorPosition(0, 0, 10); 
    turretSubsystem.turretDrive.getSensorCollection().setQuadraturePosition(0, 10);
    elevatorSubsystem.elevatorWinch.setNeutralMode(NeutralMode.Coast);
    
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    boolean sensor1 = indexerSubsystem.Sensor1.get();
    boolean sensor2 = indexerSubsystem.Sensor2.get();
    boolean sensor3 = indexerSubsystem.Sensor3.get();
    indexerSubsystem.indexStage1_2.follow(indexerSubsystem.indexStage1_1);
    indexerSubsystem.indexStage1_1.setInverted(true);
    SmartDashboard.putNumber("ball count", ballCount);
    SmartDashboard.putNumber("state change count", stateChangeCount);

    if (sensor1 == false) {
      indexerSubsystem.indexStage1_1.set(ControlMode.PercentOutput, 0.75);
      indexerSubsystem.indexKicker.set(ControlMode.PercentOutput, 0.75);
    } 
    
    else if (sensor2 == false) {
      indexerSubsystem.indexStage1_1.set(ControlMode.PercentOutput, 0);
      indexerSubsystem.indexKicker.set(ControlMode.PercentOutput, 0);
    }

    if (sensor1 != sensor1Last && sensor1 == false) {
      ballCount += 1;
      sensor1Last = sensor1;
    }

    if (sensor1 != sensor1Last && sensor1 == true) {
      sensor1Last = sensor1;
    }

    if (sensor3 != sensor3Last && sensor3 == false) {
      ballCount -= 1;
      sensor3Last = sensor3;
    }

    if (sensor3 != sensor3Last && sensor3 == true) {
      sensor3Last = sensor3;
    }

    if ((ballCount >= 1) && sensor1 == true && sensor2 == true) {
      indexerSubsystem.indexStage1_1.set(ControlMode.PercentOutput, 0.75);
    }

    if (sensor2 != sensor2Last && sensor2 == false) {
      stateChangeCount += 1;
      sensor2Last = sensor2;
    }

    if (sensor2 != sensor2Last && sensor2 == true) {
      stateChangeCount += 1;
      sensor2Last = sensor2;
    }
    
    if (ballCount == 1 && stateChangeCount != 1) {
      indexerSubsystem.indexStage1_1.set(ControlMode.PercentOutput, 0.75);
      indexerSubsystem.indexKicker.set(ControlMode.PercentOutput, 0.75);
    }
    
    if (ballCount == 2 && stateChangeCount != 3) {
      indexerSubsystem.indexStage1_1.set(ControlMode.PercentOutput, 0.75);
      indexerSubsystem.indexKicker.set(ControlMode.PercentOutput, 0.75);
      if (ballCount == 2 && stateChangeCount == 3) {
        indexerSubsystem.indexStage1_1.set(ControlMode.PercentOutput, 0);
        indexerSubsystem.indexKicker.set(ControlMode.PercentOutput, 0);
      }
    }
    
    if (ballCount == 3 && stateChangeCount != 5) {
      indexerSubsystem.indexStage1_1.set(ControlMode.PercentOutput, 0.75);
      indexerSubsystem.indexKicker.set(ControlMode.PercentOutput, 0.75);
      if (ballCount == 3 && stateChangeCount == 5) {
        indexerSubsystem.indexStage1_1.set(ControlMode.PercentOutput, 0);
        indexerSubsystem.indexKicker.set(ControlMode.PercentOutput, 0);
      }
    }

    if (ballCount == 4 && stateChangeCount != 7) {
      indexerSubsystem.indexStage1_1.set(ControlMode.PercentOutput, 0.75);
      indexerSubsystem.indexKicker.set(ControlMode.PercentOutput, 0.75);
      if (ballCount == 4 && stateChangeCount == 7) {
        indexerSubsystem.indexStage1_1.set(ControlMode.PercentOutput, 0);
        indexerSubsystem.indexKicker.set(ControlMode.PercentOutput, 0);
      }
    }

    if (ballCount == 5 && stateChangeCount != 9) {
      indexerSubsystem.indexStage1_1.set(ControlMode.PercentOutput, 0.75);
      indexerSubsystem.indexKicker.set(ControlMode.PercentOutput, 0.75);
      if (ballCount == 5 && stateChangeCount == 9) {
        indexerSubsystem.indexStage1_1.set(ControlMode.PercentOutput, 0);
        indexerSubsystem.indexKicker.set(ControlMode.PercentOutput, 0);
      }
    }
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }
}
