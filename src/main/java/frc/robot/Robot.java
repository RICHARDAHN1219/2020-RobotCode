package frc.robot;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotContainer;
import frc.robot.commands.hoodDeployCommand;
import frc.robot.commands.hoodRetractCommand;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public static boolean manualMode = false;
  public static boolean turretHome = false;
  public char stage2ColorChar = 'U';
  private RobotContainer m_robotContainer;
  public PowerDistributionPanel m_pdp = new PowerDistributionPanel();
  public Compressor Compressor;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    SmartDashboard.putNumber("distance", 0);
    SmartDashboard.putData("Hood Deploy", new hoodDeployCommand());
    SmartDashboard.putData("Hood Retract", new hoodRetractCommand());
    RobotContainer.m_limelight.setLEDMode(1);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putBoolean("limelight on target", RobotContainer.limelightOnTarget);
    double distance = RobotContainer.m_limelight.getDist(0.6096, 2.5019, 32);
    SmartDashboard.putNumber("distance", distance);
  }

  @Override
  public void disabledInit() {
    RobotContainer.m_limelight.setLEDMode(1);
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {

    // TODO: move this to RobotInit() that get's run when the robot is powered on instead of here when
    // Autonomous starts. Auton command geneneration can take almost a second. Don't waste it during a 
    // match.
    m_autonomousCommand = m_robotContainer.rightSideSingleTrenchPickupShoot4();

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
      // TODO: each of those commands is going to re-create a new command, the ones with trajectories will take a non-trivial amount of time
      // maybe create the them in RobotContainer and save them to a variable?
      CommandGroupBase.clearGroupedCommand(m_robotContainer.straightOnGoalBackUpShoot3());
      CommandGroupBase.clearGroupedCommand(m_robotContainer.rightSideSingleTrenchPickupShoot4());
      CommandGroupBase.clearGroupedCommand(m_robotContainer.middleBackUpShoot3());
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
    RobotContainer.m_shooter.testMode();
  }
}