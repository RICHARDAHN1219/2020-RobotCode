package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public static boolean manualMode = false;
  public static boolean turretHome = false;
  public static double temp; 
  private RobotContainer m_robotContainer;
  public static SupplyCurrentLimitConfiguration m_currentlimitMain = new SupplyCurrentLimitConfiguration(true, 35, 1, 1);
  public static SupplyCurrentLimitConfiguration m_currentlimitSecondary = new SupplyCurrentLimitConfiguration(true, 25, 1, 1);
  public PowerDistributionPanel m_pdp = new PowerDistributionPanel();

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

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
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      System.out.println("Scheduling Autonomous Command");
      m_autonomousCommand.schedule();
      System.out.println("Finished Autonomous Command");
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      System.out.println("Cancelling Autonomous Command");
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    //RobotContainer.m_shooter.testMode();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }
}
