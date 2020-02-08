package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public static boolean manualMode = false;
  public static boolean turretHome = false;
  public static double IMUHeading;
  public static double temp; 
  // TODO: fix me, PigeonIMU(turretSubsystem.turretDrive) can't go here;
  public static PigeonIMU m_pigeon = new PigeonIMU(20);
  @SuppressWarnings("unused")
  private RobotContainer m_robotContainer;
  public static SupplyCurrentLimitConfiguration m_currentlimitMain = new SupplyCurrentLimitConfiguration(true, 35, 1, 1);
  public static SupplyCurrentLimitConfiguration m_currentlimitSecondary = new SupplyCurrentLimitConfiguration(true, 25, 1, 1);

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
    RobotContainer.m_shooter.testMode();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }
}
