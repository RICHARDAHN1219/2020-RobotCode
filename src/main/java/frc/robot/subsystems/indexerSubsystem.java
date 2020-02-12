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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.indexConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.subsystems.blinkin;


// TODO work on conditions where we run or don't run the kicker motor
public class indexerSubsystem extends SubsystemBase {

  private WPI_TalonSRX indexIntake = new WPI_TalonSRX(indexConstants.indexIntake);
  private WPI_TalonFX indexBelts = new WPI_TalonFX(indexConstants.indexBelts);
  private WPI_TalonFX indexKicker = new WPI_TalonFX(indexConstants.indexKicker);
  public DigitalInput Sensor1 = new DigitalInput(0);
  public DigitalInput Sensor2 = new DigitalInput(1);
  private DigitalInput Sensor3 = new DigitalInput(2);
  private boolean ballReady4IndexerLast = false;
  private boolean ballStagedLast = false;
  private boolean ballExitingLast = false;
  public boolean ballReady4Indexer;
  public boolean ballStaged;
  public static boolean eject = false;
  public int stateChangeCount = 0;
  private int exitStateChangeCount = 0;
  public int ballCount = 0;
  public int restageState = 0;
  public boolean periodic = true;
  public int restageEndBallCount;
  private blinkin m_blinkin = RobotContainer.m_blinkin;

  public indexerSubsystem() {
    indexBelts.configFactoryDefault();
    indexKicker.configFactoryDefault();
    indexIntake.configFactoryDefault();

    // Voltage limits, percent output is scaled to this new max
    indexBelts.configVoltageCompSaturation(11);
    indexBelts.enableVoltageCompensation(true);
    indexKicker.configVoltageCompSaturation(11);
    indexKicker.enableVoltageCompensation(true);
    indexIntake.configVoltageCompSaturation(11);
    indexIntake.enableVoltageCompensation(true);

    // current limits
    indexBelts.configSupplyCurrentLimit(Robot.m_currentlimitSecondary);
    indexKicker.configSupplyCurrentLimit(Robot.m_currentlimitSecondary);
    indexIntake.configSupplyCurrentLimit(Robot.m_currentlimitSecondary);

    // Brake mode
    indexBelts.setNeutralMode(NeutralMode.Brake);
    indexKicker.setNeutralMode(NeutralMode.Brake);
    indexIntake.setNeutralMode(NeutralMode.Brake);

    // Invert
    indexBelts.setInverted(false);
    indexKicker.setInverted(false);
    indexIntake.setInverted(false);

    indexBelts.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
    indexKicker.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
    indexIntake.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    // TalonFX don't have sensor phase only TalonSRX
    indexIntake.setSensorPhase(false);
    
    //Set Ramp-Up
    //indexKicker.configClosedloopRamp(0.1);
    //indexBelts.configClosedloopRamp(0.1);
    //indexIntake.configClosedloopRamp(0.1);


    // Config PID values to control RPM
    // TODO: test PID values
    indexBelts.config_kP(0, 0.15, 10);
    indexBelts.config_kI(0, 0.0, 10);
    indexBelts.config_kD(0, 1.5, 10);
    indexBelts.config_kF(0, 0.048, 10);

    indexKicker.config_kP(0, 0.15, 10);
    indexKicker.config_kI(0, 0.0, 10);
    indexKicker.config_kD(0, 1.5, 10);
    indexKicker.config_kF(0, 0.053, 10);

    indexIntake.config_kP(0, 0.1, 10);
    indexIntake.config_kI(0, 0.0, 10);
    indexIntake.config_kD(0, 0.0, 10);
    indexIntake.config_kF(0, 0.0, 10);

    // Note: if we add position control, then we need to add a second set of PID parameters
    // on PID index 1, and then swtich between the Talon PID index when setting RPM and Position
  }

  @Override
  public void periodic() {
    boolean ballReady4Indexer = !Sensor1.get();
    boolean ballStaged = !Sensor2.get();
    boolean ballExiting = !Sensor3.get();
    SmartDashboard.putNumber("ball count", ballCount);
    SmartDashboard.putNumber("state change count", stateChangeCount);
    SmartDashboard.putNumber("restage state", restageState);
    SmartDashboard.putNumber("Belt RPM", indexBelts.getSelectedSensorVelocity() * 600 / 2048);
    SmartDashboard.putNumber("Kicker RPM", indexKicker.getSelectedSensorVelocity() * 600 / 2048);
    SmartDashboard.putNumber("Intake RPM", indexIntake.getSelectedSensorVelocity() * 600 / 2048);

    // prevent intake of new balls when we already have 5
    // EDIT: We are aiming for 4 balls by week 1 until 5 is figured out
    if (periodic == true) {
      if (Robot.manualMode == true) {
        setIntakePercentOutput((RobotContainer.m_operatorController.getTriggerAxis(Hand.kRight)
            - RobotContainer.m_operatorController.getTriggerAxis(Hand.kLeft)) * 0.6);
        setBeltsPercentOutput(RobotContainer.m_operatorController.getTriggerAxis(Hand.kRight)
            - RobotContainer.m_operatorController.getTriggerAxis(Hand.kLeft));
        setKickerPercentOutput(RobotContainer.m_operatorController.getTriggerAxis(Hand.kRight)
            - RobotContainer.m_operatorController.getTriggerAxis(Hand.kLeft));
      } else {

        // move indexer when a new ball is ready to enter the system
        if (ballReady4Indexer == true) {
          setIntakePercentOutput(0.6);
          setBeltsPercentOutput(1);
          setKickerPercentOutput(0.3);
          m_blinkin.solid_blue();
        }

        // stop indexer when balls are properly staged
        else if (ballStaged == true) {
          setBeltsPercentOutput(0);
          m_blinkin.solid_red();
        }

        // finish staging balls when this error state occurs
        if ((-1 + (ballCount * 2)) != stateChangeCount) {
          setIntakePercentOutput(0.6);
          setBeltsPercentOutput(1);
          setKickerPercentOutput(0.3);
          m_blinkin.solid_blue();
        }

        // finish staging balls when this error state occurs
        if ((ballCount >= 1) && ballReady4Indexer == false && ballStaged == false) {
          setIntakePercentOutput(0.6);
          setBeltsPercentOutput(1);
          setKickerPercentOutput(0.3);
          m_blinkin.solid_blue();
        }

        // automatically stage the balls for shooting when we have 4
        if (ballCount == 4 && ballExiting == false) {
          setIntakePercentOutput(0.6);
          setBeltsPercentOutput(1);
          m_blinkin.solid_green_lime();
        }

        // stop indexer when all 4 balls are staged for shooting
        else if (ballCount == 4 && ballExiting == true) {
          setIntakePercentOutput(0);
          setBeltsPercentOutput(0);
          m_blinkin.solid_pink();
        }

        if (ballExiting == true) {
          setIntakePercentOutput(0);
          setBeltsPercentOutput(0);
          setKickerPercentOutput(0);
          m_blinkin.solid_red();
        }

        if (eject == true) {
          // setBeltsPercentOutput(1);
          // setKickerPercentOutput(1);
          setIntakePercentOutput(0.6);
          setBeltsRPM(6380);
          setKickerRPM(6380);
          m_blinkin.solid_green();
        }
      }
    } else {

      // increase ball count as balls enter the indexer
      if (ballReady4Indexer != ballReady4IndexerLast && ballReady4Indexer == true) {
        ballCount += 1;
      }
      ballReady4IndexerLast = ballReady4Indexer;

      // count number of state changes on ballStaged sensor to combat error states
      if (ballStaged != ballStagedLast) {
        stateChangeCount += 1;
        ballStagedLast = ballStaged;
      }
    }

    // decrease ballCount as balls leave the indexer
    if (ballExiting != ballExitingLast && ballExiting == false) {
      ballCount -= 1;
      stateChangeCount = stateChangeCount - 2;
    }
    ballExitingLast = ballExiting;

    // count number of state changes as balls leave the system
    if (ballExiting != ballExitingLast) {
      exitStateChangeCount += 1;
      ballExitingLast = ballExiting;
    }

    // don't let state changes go below zero
    if (stateChangeCount < 0) {
      stateChangeCount = 0;
    }

    // can't have negative balls in the robot
    if (ballCount == 0) {
      stateChangeCount = 0;
    }
  }

  public void feedOneBall() {
    final int singleFeedInitialStateCount = exitStateChangeCount;
    final int singleFeedExitStateCount = singleFeedInitialStateCount + 2;

    if (exitStateChangeCount != singleFeedExitStateCount) {
      setIntakePercentOutput(0.6);
      setBeltsPercentOutput(1);
      setKickerPercentOutput(1);
    } else {
      return;
    }
  }

  public void setBeltsPercentOutput(double percent) {
    indexBelts.set(ControlMode.PercentOutput, percent);
  }

  public void setKickerPercentOutput(double percent) {
    indexKicker.set(ControlMode.PercentOutput, percent);
  }

  public void setIntakePercentOutput(double percent) {
    indexIntake.set(ControlMode.PercentOutput, percent);
  }

  public void setBeltsRPM(double rpm) {
    indexBelts.set(ControlMode.Velocity, rpm * 2048 / 600);
  }

  public void setKickerRPM(double rpm) {
    indexKicker.set(ControlMode.Velocity, rpm * 2048 / 600);
  }

  public void setIntakeRPM(double rpm) {
    indexIntake.set(ControlMode.Velocity, rpm * 2048 / 600);
  }

  /** 
   * Stop all motors
   */
  public void stop() {
    setBeltsPercentOutput(0.0);
    setKickerPercentOutput(0.0);
    setIntakePercentOutput(0.0);
    m_blinkin.solid_orange());
  }

  /**
   * ballReadyForIndexer - monitor sensor 1 for a ball ready to be indexed.
   * 
   * @return true if a ball is waiting to be indexed
   */
  public boolean ballReadyForIndexer() {
    return ! Sensor1.get();
  }

  /**
   * runIntake() - pull a ball into the indexer
   */
  public void runIntake() {
    setBeltsRPM(6380);
    setKickerRPM(0);
    setIntakePercentOutput(0.6);
    m_blinkin.solid_green();
  }

  /**
   * runKicker() - eject ball from indexer
   */
  public void runKicker() {
    setBeltsRPM(6380);
    setKickerRPM(6380);
    setIntakePercentOutput(0.6);
    m_blinkin.solid_pink();
  }

  /**
   * runBelts() - run only the belts
   */
  public void runBelts() {
    setBeltsRPM(6380);
    setKickerRPM(0.0);
    setIntakePercentOutput(0.0);
    m_blinkin.solid_blue();
  }

  /**
   * reverseIntake() - run intake in reverse
   */
  public void reverseIntake() {
    setBeltsRPM(-6000);
    setKickerRPM(0);
    setIntakePercentOutput(-0.7);
    m_blinkin.strobe_red();
  }

  /**
   * return true of ball is in the staged position
   * 
   * @return boolean
   */
  public boolean ballIndexed() {
    return ! Sensor2.get();
  }

  /**
   * return true of ball waiting for 
   * 
   * @return boolean
   */
  public boolean indexerFull() {
    return ! Sensor3.get();
  }

  public static void setEject(boolean e){
    eject = e;
  }

  /**
   * getBallCount() return the number of balls in the indexer
   * 
   * @return int ball count
   */
  public int getBallCount() {
    return ballCount;
  }

}
