/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team2930.lib.util.linearInterpolator;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.shooterConstants;

public class shooterSubsystem extends SubsystemBase {

  private CANSparkMax neo_shooter1 = new CANSparkMax(shooterConstants.shooter1, MotorType.kBrushless);
  private CANSparkMax neo_shooter2 = new CANSparkMax(shooterConstants.shooter2, MotorType.kBrushless);
  private Solenoid hood = new Solenoid(shooterConstants.shooterHood);
  private CANPIDController m_pidController;
  private CANEncoder m_encoder;
  private double kMaxOutput, kMinOutput;
  private double m_desiredRPM = 0;
  private boolean m_atSpeed = false;
  private long m_initalTime = 0;
  private linearInterpolator m_lt;
  
  private double hoodDownP[][] = {
    {15.4, 2600}, // 4.5 feet
    {3, 2650}, // 7 feet
    {-7.2, 2750}, // 10 feet
    {-12.2, 2900} // 12 feet
  };
  private double hoodUpP[][] = {
    {8, 3900}, // 9 feet
    {-0.1, 3550}, // 13 feet
    {-5, 3600}, // 17 feet
    {-8.5, 3800}, // 21 feet
    {-11, 4100} // 25 feet
  };

  private double hoodDownC[][] = {
    {15.4, 2600}, // 4.5 feet
    {3, 2650}, // 7 feet
    {-7.2, 2750}, // 10 feet
    {-12.2, 2900} // 12 feet
  };
  private double hoodUpC[][] = {
    {8, 3900}, // 9 feet
    {-0.1, 3550}, // 13 feet
    {-5, 3600}, // 17 feet
    {-8.5, 3800}, // 21 feet
    {-11, 4100} // 25 feet
  };

  public shooterSubsystem() {
    neo_shooter1.restoreFactoryDefaults();
    neo_shooter2.restoreFactoryDefaults();

    //Current Limits for use on competition bot
    //neo_shooter1.setSmartCurrentLimit(35);
    //neo_shooter2.setSmartCurrentLimit(35);
    
    // Set coast mode
    neo_shooter1.setIdleMode(CANSparkMax.IdleMode.kCoast);
    neo_shooter2.setIdleMode(CANSparkMax.IdleMode.kCoast);
    
    if (Robot.isCompBot == true) {
      neo_shooter1.setInverted(true);
    }

    neo_shooter2.follow(neo_shooter1, true);
    m_pidController = neo_shooter1.getPIDController();
    m_encoder = neo_shooter1.getEncoder(EncoderType.kHallSensor, 4096);
    kMaxOutput = 1; 
    kMinOutput = -1;
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    SmartDashboard.putNumber("ShooterRPM", m_desiredRPM);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("ActualShooterRPM", (int) m_encoder.getVelocity());

    double rpm = SmartDashboard.getNumber("ShooterRPM", -1);
    if (rpm != -1) {
      if (rpm == 0.0) {
        // spin down, don't use PID (and power) to stop
        stop();
      }
      else if (m_desiredRPM != rpm ) {
        setShooterRPM(rpm);
        System.out.println("New shooter desired RPM: "  + rpm);
        m_initalTime = System.nanoTime();
        m_atSpeed = false;
      }
    }

    m_initalTime = System.nanoTime();
    if (isAtSpeed()) {
      if (!m_atSpeed) {
        SmartDashboard.putNumber("Time2RPM", System.nanoTime() - m_initalTime);
      }
      m_atSpeed = true;
    }
    else {
      m_atSpeed = false;
      m_initalTime = System.nanoTime();
    }
    SmartDashboard.putBoolean("isAtSpeed", m_atSpeed);
  }

  public void setShooterRPM (double desiredRPM) {
    m_desiredRPM = desiredRPM;
    m_pidController.setReference(desiredRPM, ControlType.kVelocity);
    SmartDashboard.putNumber("ShooterRPM", m_desiredRPM);
  }

  public void testMode(){
    m_desiredRPM = SmartDashboard.getNumber("DesiredShooterRPM", 0);
    System.out.println("Shooter desired ROM: "  + m_desiredRPM);
    m_pidController.setReference(m_desiredRPM, ControlType.kVelocity);
    System.out.println("Activating Test Mode");
  }

  // TODO: put pipeline setting in if statement if we make different comp bot pipelines
  public void deployHood() {
    RobotContainer.m_limelight.setPipeline(5);
    if (Robot.isCompBot == true) {
      setShooterPID(0.0005, 0.00000025, 0, 0.00022, 250);
      m_lt = new linearInterpolator(hoodUpC);
    }
    else {
      setShooterPID(0.0005, 0.00000025, 0, 0.00022, 250);
      m_lt = new linearInterpolator(hoodUpP);
    }
    hood.set(true);
  }

  // TODO: put pipeline setting in if statement if we make different comp bot pipelines
  public void retractHood() {
    RobotContainer.m_limelight.setPipeline(4);
    if (Robot.isCompBot == true) {
      setShooterPID(0.0004, 0.00000025, 0, 0.0002, 250);
      m_lt = new linearInterpolator(hoodDownC);
    }
    else {
      setShooterPID(0.0004, 0.00000025, 0, 0.0002, 250);
      m_lt = new linearInterpolator(hoodDownP);
    }
    hood.set(false);
  }

  public void setShooterPID (double P, double I, double D, double F, double iZ) {
    m_pidController.setP(P);
    m_pidController.setI(I);
    m_pidController.setD(D);
    m_pidController.setFF(F);
    m_pidController.setIZone(iZ);
  }

  //Current limiting on the fly switching removed due to the SparkMAX API not supporting that sort of switch.
  public void setPercentOutput(double percent) {
    neo_shooter1.set(percent);
  }

  public boolean isAtSpeed(){
    double error = m_desiredRPM - m_encoder.getVelocity();
    SmartDashboard.putNumber("RPM_Error", error);
    if (Math.abs(error) < 50) {
      return true;
    } else {
      return false;
    }
  }

  /**
   * getRPMforTY() - return RPM based on limelight TY value
   * 
   * @param TY limelight TY value
   * @return RPM for flywheel
   */
  public double getRPMforTY(double TY) {
    return m_lt.getInterpolatedValue(TY);
  }

  /**
   * getRPMforDistanceFeet() - return RPM based on distance to target in FEET
   * 
   * @param distanceFeet distance in FEET to goal
   * @return RPM for flywheel
   */
  public double getRPMforDistanceFeet(double distanceFeet) {
    return m_lt.getInterpolatedValue(distanceFeet);
  }

    /**
   * getRPMforDistanceFeet() - return RPM based on distance to target in METERS
   * 
   * @param distanceFeet distance in METERS to goal
   * @return RPM for flywheel
   */
  public double getRPMforDistanceMeter(double distanceMeters) {
    return getRPMforDistanceFeet(distanceMeters * 3.28084);
  }

  public void stop() {
    m_desiredRPM = 0;
    setPercentOutput(0.0);
  }
}
