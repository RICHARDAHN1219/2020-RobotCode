package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.elevatorSubsystem;
import frc.robot.subsystems.turretSubsystem;
import com.ctre.phoenix.motorcontrol.*;
import edu.wpi.first.wpilibj.DriverStation;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public static boolean manualMode = false;
  public static boolean turretHome = false;
  public static double IMUHeading;
  public static double temp;
  public char stage2ColorChar = 'U';
  // TODO: fix me, PigeonIMU(turretSubsystem.turretDrive) can't go here;
  public static PigeonIMU m_pigeon = new PigeonIMU(20);
  @SuppressWarnings("unused")
  private RobotContainer m_robotContainer;
  public static SupplyCurrentLimitConfiguration m_currentlimitMain = new SupplyCurrentLimitConfiguration(true, 35, 1,
      1);
  public static SupplyCurrentLimitConfiguration m_currentlimitSecondary = new SupplyCurrentLimitConfiguration(true, 25,
      1, 1);

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    temp = m_pigeon.getTemp();
    IMUHeading = m_pigeon.getFusedHeading();
    SmartDashboard.putNumber("CompassFieldStrength", m_pigeon.getCompassFieldStrength());
    SmartDashboard.putNumber("IMU Fused Heading", IMUHeading);
    SmartDashboard.putNumber("Temperature (VERY IMPORTANT)", temp);
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
    m_pigeon.setFusedHeadingToCompass();
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

    //Does the target color also have to be a number 
    String gameData;
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    if (gameData.length() > 0) {
      switch (gameData.charAt(0)) {
      case 'B':
        // Blue case code
       stage2ColorChar = 'B';
        break;
      case 'G':
        // Green case code
       stage2ColorChar = 'G';
        break;
      case 'R':
        // Red case code
        stage2ColorChar = 'R';
        break;
      case 'Y':
        // Yellow case code
        stage2ColorChar = 'Y';
        break;
      default:
        // This is corrupt data
        break;
      }
    } else {
      // Code for no data received yet
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }
}
