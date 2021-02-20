package frc.robot;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotContainer;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public static boolean manualMode = false;
  public static boolean turretHome = false;
  public char stage2ColorChar = 'U';
  private RobotContainer m_robotContainer;
  public PowerDistributionPanel m_pdp = new PowerDistributionPanel();
  public Compressor Compressor;
  public static boolean isCompBot = true;
  SendableChooser <String> chooser = new SendableChooser<>();

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    SmartDashboard.putNumber("distance", 0);
    //RobotContainer.m_limelight.setLEDMode(1);
    CameraServer.getInstance().startAutomaticCapture();
    //chooser.addOption("Right 3 Ball", "r3");
    //chooser.addOption("Right 4 ball", "r4");
    //chooser.addOption("Right 5 ball", "r5");
    //chooser.addOption("Right 6 ball", "r6");
    //chooser.addOption("Right 6 ball test", "r6t");
    //chooser.addOption("straight on 3", "s3");
    //chooser.addOption("straight on 3 forward", "s3f");
    chooser.addOption("right side 6 reorg", "r6reorg");
    chooser.addOption("Center 3 ball", "m3");
    chooser.addOption("straight 1m", "s1m");
    chooser.addOption("straight 1m, right 1m", "s1r1m");
    chooser.addOption("straight 1m, back 1m", "s1b1m");
    chooser.addOption("circle", "circle");
    chooser.addOption("figure 8", "figure8");
    chooser.setDefaultOption("do nothing", "nothing");
    SmartDashboard.putData("Auto mode", chooser);
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
    //RobotContainer.m_limelight.setLEDMode(1);
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonCommand(chooser.getSelected());
    
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