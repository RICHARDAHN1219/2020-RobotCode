/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.fearxzombie.limelight;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team2930.lib.util.linearInterpolator;

import frc.robot.Constants.shooterConstants;

public class shooterSubsystem extends SubsystemBase {

  private CANSparkMax neo_shooter1 = new CANSparkMax(shooterConstants.shooter1, MotorType.kBrushless);
  private CANSparkMax neo_shooter2 = new CANSparkMax(shooterConstants.shooter2, MotorType.kBrushless);
  private CANPIDController m_pidController;
  private CANEncoder m_encoder;
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private double m_desiredRPM = 0;
  private boolean m_atSpeed = false;
  private long m_initalTime = 0;
  private linearInterpolator m_lt;
  private double dist = -1;
  private boolean oneXZoom;
  private boolean twoXZoom;
  private boolean threeXZoom;
  private boolean lock;
  limelight m_limelight;
  private double data[][] = {
    // distance in Feed -> RPM
    { 4,  2650 }, 
    { 5,  2550 },
    { 6,  2550 },
    { 7,  2550 },
    { 8,  2600 },
    { 9,  2650 },
    { 10, 2700 },
    
    { 25, 4300},
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

    neo_shooter2.follow(neo_shooter1, true);
    m_pidController = neo_shooter1.getPIDController();
    m_encoder = neo_shooter1.getEncoder(EncoderType.kHallSensor, 4096);
    kMaxOutput = 1; 
    kMinOutput = -1;
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    setShooterPID(0.0003, 0.00000025, 0, 0.0002, 250);

    SmartDashboard.putNumber("ShooterRPM", m_desiredRPM);
    SmartDashboard.putNumber("UpdatedRPM", -1);

    m_lt = new linearInterpolator(data);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ActualShooterRPM", (int) m_encoder.getVelocity());
    //m_desiredRPM = SmartDashboard.getNumber("DesiredShooterRPM", 0);
    double rpm = SmartDashboard.getNumber("ShooterRPM", -1);
    if (rpm != -1) {
      if (m_desiredRPM != rpm ) {
        setShooterRPM(rpm);
        System.out.println("New shooter desired RPM: "  + m_desiredRPM);
        // lets' confirm we're changing this
        SmartDashboard.putNumber("UpdatedRPM", m_desiredRPM);
        m_initalTime = System.nanoTime();
        m_atSpeed = false;
      }
      if (m_limelight.getTV() == 1) {
        lock = true;
      } else {
        lock = false;
      }
      // This method will be called once per scheduler run
          //Example output: currentDist = (2.5019-0.6096 / tan(10+20))
      //currentDist = 3.27755 so 3x Zoom shall be used.
      double h1 = 0.6096;
      double h2 = 2.5019;  // TODO: This is wrong, update never got committed
      double a1 = 32;
      double a2 = m_limelight.getTY();
      double oneXDist = 1.7272;
      double twoXDist = 2.7178;
      double threeXDist = 2.9718;
      double currentDist = ((Math.abs(h2 - h1) / Math.tan((a1 + a2) * Math.PI / 180)) / 1.1154856);
      // TODO: figure out whey we need a fudge factor?
      SmartDashboard.putBoolean("1xZoom", oneXZoom);
      SmartDashboard.putBoolean("2xZoom", twoXZoom);
      SmartDashboard.putBoolean("3xZoom", threeXZoom);
      SmartDashboard.putBoolean("LL_TARGETLOCK", lock);
      dist = currentDist;
    //  if (currentDist >= oneXDist || getTV() == 1){
    //     set1xZoom();
    //     System.out.println("Switching to 1x Zoom");
    //   } else if (currentDist >= twoXDist || getTV() == 1){
    //     set2xZoom();
    //     System.out.println("Switching to 2x Zoom");
    //   } else if (currentDist >= threeXDist || getTV() == 1) {
    //     set3xZoom();
    //     System.out.println("Switching to 3x Zoom");
    //   } if (a2 == 0 || getTV() == 0){
    //     dist = -1;
    //     set1xZoom();
    //     System.out.println("No target in range, switching to normal zoom.");
    //   }
      
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
  }

  public void testMode(){
    //m_desiredRPM = SmartDashboard.getNumber("DesiredShooterRPM", 0);
    //System.out.println("Shooter desired ROM: "  + m_desiredRPM);
    //m_pidController.setReference(m_desiredRPM, ControlType.kVelocity);
    //System.out.println("Activating Test Mode");
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
    if (Math.abs(error) < 100){
      return true;
    } else {
      return false;
    }
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
    setPercentOutput(0.0);
  }
}
