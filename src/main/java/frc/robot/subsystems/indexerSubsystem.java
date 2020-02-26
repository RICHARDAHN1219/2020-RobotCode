/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.indexConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.currentLimits;
import static frc.robot.Constants.digitalIOConstants.dio0_indexerSensor1;
import static frc.robot.Constants.digitalIOConstants.dio1_indexerSensor2;
import static frc.robot.Constants.digitalIOConstants.dio2_indexerSensor3;

public class indexerSubsystem extends SubsystemBase {

  private WPI_TalonSRX indexIntakeP = new WPI_TalonSRX(indexConstants.indexIntake);
  private WPI_VictorSPX indexIntakeC = new WPI_VictorSPX(indexConstants.indexIntake);
  private WPI_TalonFX indexBelts = new WPI_TalonFX(indexConstants.indexBelts);
  private WPI_TalonFX indexKickerP = new WPI_TalonFX(indexConstants.indexKicker);
  private WPI_VictorSPX indexKickerC = new WPI_VictorSPX(indexConstants.indexKicker);
  private DigitalInput Sensor1 = new DigitalInput(dio0_indexerSensor1);
  private DigitalInput Sensor2 = new DigitalInput(dio1_indexerSensor2);
  private DigitalInput Sensor3 = new DigitalInput(dio2_indexerSensor3);
  private boolean ballReady4IndexerLast = false;
  private boolean ballExitingLast = false;
  private boolean ballReady4Indexer;
  private boolean ballStaged;
  private boolean ballExiting;
  private int ballCount = 0;
  private int restageState = 0;
  private blinkinSubsystem m_blinkin = RobotContainer.m_blinkin;
  private boolean finishedSingleFeed;

  public indexerSubsystem() {
    indexBelts.configFactoryDefault();
    indexKickerP.configFactoryDefault();
    indexKickerC.configFactoryDefault();
    indexIntakeP.configFactoryDefault();
    indexIntakeC.configFactoryDefault();

    // Voltage limits, percent output is scaled to this new max
    indexBelts.configVoltageCompSaturation(11);
    indexBelts.enableVoltageCompensation(true);
    indexKickerP.configVoltageCompSaturation(11);
    indexKickerP.enableVoltageCompensation(true);
    indexKickerC.configVoltageCompSaturation(11);
    indexKickerC.enableVoltageCompensation(true);
    indexIntakeP.configVoltageCompSaturation(11);
    indexIntakeP.enableVoltageCompensation(true);
    indexIntakeC.configVoltageCompSaturation(11);
    indexIntakeC.enableVoltageCompensation(true);

    // current limits
    indexBelts.configSupplyCurrentLimit(currentLimits.m_currentlimitSecondary);
    indexKickerP.configSupplyCurrentLimit(currentLimits.m_currentlimitSecondary);
    indexIntakeP.configSupplyCurrentLimit(currentLimits.m_currentlimitSecondary);

    // Brake mode
    indexBelts.setNeutralMode(NeutralMode.Brake);
    indexKickerP.setNeutralMode(NeutralMode.Brake);
    indexKickerC.setNeutralMode(NeutralMode.Brake);
    indexIntakeP.setNeutralMode(NeutralMode.Brake);
    indexIntakeC.setNeutralMode(NeutralMode.Brake);

    // Invert
    indexBelts.setInverted(false);
    indexKickerP.setInverted(false);
    indexKickerC.setInverted(false);
    indexIntakeP.setInverted(true);
    indexIntakeC.setInverted(true);

    indexBelts.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
    indexKickerP.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
    indexKickerC.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
    indexIntakeP.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    indexIntakeC.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    // TalonFX don't have sensor phase only TalonSRX
    indexIntakeP.setSensorPhase(false);
    indexIntakeC.setSensorPhase(false);
    indexKickerC.setSensorPhase(false);
    
    //Set Ramp-Up
    //indexKicker.configClosedloopRamp(0.1);
    //indexBelts.configClosedloopRamp(0.1);
    //indexIntake.configClosedloopRamp(0.1);


    // Config PID values to control RPM
    indexBelts.config_kP(0, 0.15, 10);
    indexBelts.config_kI(0, 0.0, 10);
    indexBelts.config_kD(0, 1.5, 10);
    indexBelts.config_kF(0, 0.048, 10);

    indexKickerP.config_kP(0, 0.15, 10);
    indexKickerP.config_kI(0, 0.0, 10);
    indexKickerP.config_kD(0, 1.5, 10);
    indexKickerP.config_kF(0, 0.053, 10);

    indexKickerC.config_kP(0, 0.15, 10);
    indexKickerC.config_kI(0, 0.0, 10);
    indexKickerC.config_kD(0, 1.5, 10);
    indexKickerC.config_kF(0, 0.053, 10);

    indexIntakeP.config_kP(0, 0.1, 10);
    indexIntakeP.config_kI(0, 0.0, 10);
    indexIntakeP.config_kD(0, 0.0, 10);
    indexIntakeP.config_kF(0, 0.0, 10);

    indexIntakeC.config_kP(0, 0.1, 10);
    indexIntakeC.config_kI(0, 0.0, 10);
    indexIntakeC.config_kD(0, 0.0, 10);
    indexIntakeC.config_kF(0, 0.0, 10);

    // Note: if we add position control, then we need to add a second set of PID parameters
    // on PID index 1, and then swtich between the Talon PID index when setting RPM and Position
  }

  @Override
  public void periodic() {
    boolean ballReady4Indexer = !Sensor1.get();
    boolean ballStaged = !Sensor2.get();
    boolean ballExiting = !Sensor3.get();
    SmartDashboard.putNumber("ball count", ballCount);
    SmartDashboard.putNumber("restage state", restageState);
    SmartDashboard.putNumber("Belt RPM", indexBelts.getSelectedSensorVelocity() * 600 / 2048);
    if (Robot.isCompBot == true) {
      SmartDashboard.putNumber("Kicker RPM", indexKickerC.getSelectedSensorVelocity() * 600 / 2048);
    }
    else {
      SmartDashboard.putNumber("Kicker RPM", indexKickerP.getSelectedSensorVelocity() * 600 / 2048);
    }
    
    // increase ball count as balls enter the indexer
    if (ballReady4Indexer != ballReady4IndexerLast && ballReady4Indexer == false) {
      ballCount += 1;
    }
    ballReady4IndexerLast = ballReady4Indexer;

    // decrease ballCount as balls leave the indexer
    if (ballExiting != ballExitingLast && ballExiting == false) {
      ballCount -= 1;
      
    }
    ballExitingLast = ballExiting;
  }

  public void setBeltsPercentOutput(double percent) {
    indexBelts.set(ControlMode.PercentOutput, percent);
  }

  public void setKickerPercentOutput(double percent) {
    if (Robot.isCompBot == true) {
      indexKickerC.set(ControlMode.PercentOutput, percent);
    }
    else {
      indexKickerP.set(ControlMode.PercentOutput, percent);
    }
  }

  public void setIntakePercentOutput(double percent) {
    if (Robot.isCompBot == true) {
      indexIntakeC.set(ControlMode.PercentOutput, percent);
    }
    else {
      indexIntakeP.set(ControlMode.PercentOutput, percent);
    }
  }

  public void setBeltsRPM(double rpm) {
    indexBelts.set(ControlMode.Velocity, rpm * 2048 / 600);
  }

  public void setKickerRPM(double rpm) {
    if (Robot.isCompBot == true) {
      indexKickerC.set(ControlMode.Velocity, rpm * 2048 / 600);
    }
    else {
      indexKickerP.set(ControlMode.Velocity, rpm * 2048 / 600);
    }
  }

  public void setIntakeRPM(double rpm) {
    if (Robot.isCompBot == true) {
      indexIntakeC.set(ControlMode.Velocity, rpm * 2048 / 600);
    }
    else {
      indexIntakeP.set(ControlMode.Velocity, rpm * 2048 / 600);
    }
  }

  /** 
   * Stop all motors
   */
  public void stopIndexer() {
    setBeltsPercentOutput(0.0);
    setKickerPercentOutput(0.0);
    setIntakePercentOutput(0.0);
    m_blinkin.solid_orange();
  }

  /**
   * ballReadyForIndexer - monitor sensor 1 for a ball ready to be indexed
   * 
   * @return true if a ball is waiting to be indexed
   */
  public boolean ballReadyForIndexer() {
    return ! Sensor1.get();
  }

  /**
   * ballStaged - monitor sensor 2 for a ball that is staged
   * 
   * @return true if a ball is staged
   */
  public boolean ballStaged() {
    return ! Sensor2.get();
  }

  /**
   * ballExiting - monitor sensor 3 for a ball that is at the kickers
   * 
   * @return true if a ball is at the kickers
   */
  public boolean ballExiting() {
    return ! Sensor3.get();
  }

  /**
   * runIndexer() - run all indexer motors at ball staging speeds
   */
  public void runIndexer() {
    if (Robot.isCompBot == true) {
      setIntakePercentOutput(0.7);
      setBeltsRPM(6380);
      setKickerPercentOutput(0.3);
      m_blinkin.solid_green();
    }
    else {
      setIntakePercentOutput(0.7);
      setBeltsRPM(6380);
      setKickerRPM(1914);
      m_blinkin.solid_green();
    }
  }

  /**
   * runBelts() - run only the belts
   */
  public void runOnlyBelts() {
    if (Robot.isCompBot == true) {
      setIntakePercentOutput(0);
      setBeltsRPM(6380);
      setKickerPercentOutput(0);
      m_blinkin.solid_blue();
    }
    else {
      setIntakePercentOutput(0);
      setBeltsRPM(6380);
      setKickerRPM(0);
      m_blinkin.solid_blue();
    }
  }

  /**
   * reverseIndexer() - run all indexer motors backwards at staging speeds
   */
  public void reverseIndexer() {
    if (Robot.isCompBot == true) {
      setIntakePercentOutput(-0.7);
      setBeltsRPM(-6380);
      setKickerPercentOutput(-0.3);
      m_blinkin.strobe_red();
    }
    else {
      setIntakePercentOutput(-0.7);
      setBeltsRPM(-6380);
      setKickerRPM(-1914);
      m_blinkin.strobe_red();
    }
  }

  /**
   * ejectIndexer() - run all indexer motors at eject/shooting speeds
   */
  public void ejectIndexer() {
    if (Robot.isCompBot == true) {
      setIntakePercentOutput(0.7);
      setBeltsRPM(6380);
      setKickerPercentOutput(1);
    }
    else {
      setIntakePercentOutput(0.7);
      setBeltsRPM(6380);
      setKickerRPM(6380);
    }
  }

  /**
   * runIntake() - run intake motor
   */
  public void runIntake() {
    setIntakePercentOutput(0.7);
  }

  /**
   * stopIntake() - stop intake motor
   */
  public void stopIntake() {
    setIntakePercentOutput(0);
  }
  
  /**
   * runOnlyIntake() - run intake motor and stop the belts and kicker
   */
  public void runOnlyIntake() {
    if (Robot.isCompBot == true) {
      setIntakePercentOutput(0.7);
      setBeltsRPM(0);
      setKickerPercentOutput(0);
    }
    else {
      setIntakePercentOutput(0.7);
      setBeltsRPM(0);
      setKickerRPM(0);
    }
  } 

  /**
   * stopBelts() - stop the belts motor
   */
  public void stopBelts() {
    setBeltsRPM(0);
  }

  /**
   * stopKicker() - stop the kicker motor
   */
  public void stopKicker() {
    if (Robot.isCompBot == true) {
      setKickerPercentOutput(0);
    }
    else {
      setKickerRPM(0);
    }
  }

  /**
   * getBallCount() - return the number of balls in the indexer
   * 
   * @return int ball count
   */
  public int getBallCount() {
    return ballCount;
  }

  /**
   * setBallCount() - set the number of balls in the indexer
   */
  public void setBallCount(int BallCount) {
    ballCount = BallCount;
  }

  /**
   * getRestageState() - return the state number of the restage command
   * 
   * @return int restage state
   */
  public int getRestageState() {
    return restageState;
  }

  /**
   * setRestageState() - set the restage state
   */
  public void setRestageState(int RestageState) {
    restageState = RestageState;
  }

  /**
   * setFinishedSingleFeed() - set the status of the single feed command
   */
  public void setFinishedSingleFeed(boolean finished) {
    finishedSingleFeed = finished;
  }

  /**
   * getFinishedSingleFeed() - gets the status of the single feed command
   * 
   * @return boolean finished single feed
   */
  public boolean getFinishedSingleFeed() {
    return finishedSingleFeed;
  }
}
