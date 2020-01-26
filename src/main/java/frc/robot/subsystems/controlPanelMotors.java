/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ControlPanelConstants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class controlPanelMotors extends SubsystemBase {
  /**
   * Creates a new controlPanelMotors.
   */
  private WPI_TalonFX m_motor;

  public controlPanelMotors() {
    m_motor = new WPI_TalonFX(ControlPanelConstants.kMotorPort);
    WPI_TalonFX m_motor = new WPI_TalonFX(4);
   
    /* Factory Default all hardware to prevent unexpected behaviour */
    m_motor.configFactoryDefault();

    /* Config the sensor used for Primary PID and sensor direction */
    m_motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, ControlPanelConstants.kPIDLoopIdx,
        ControlPanelConstants.kTimeoutMs);

    /* Ensure sensor is positive when output is positive */
    m_motor.setSensorPhase(ControlPanelConstants.kSensorPhase);

    /**
     * Set based on what direction you want forward/positive to be. This does not
     * affect sensor phase.
     */
    m_motor.setInverted(ControlPanelConstants.kMotorInvert);

    /* Config the peak and nominal outputs, 12V means full */
    m_motor.configNominalOutputForward(0, ControlPanelConstants.kTimeoutMs);
    m_motor.configNominalOutputReverse(0, ControlPanelConstants.kTimeoutMs);
    m_motor.configPeakOutputForward(.2, ControlPanelConstants.kTimeoutMs);
    m_motor.configPeakOutputReverse(-.2, ControlPanelConstants.kTimeoutMs);

    /**
     * Config the allowable closed-loop error, Closed-Loop output will be neutral
     * within this range. See Table in Section 17.2.1 for native units per rotation.
     */
    m_motor.configAllowableClosedloopError(0, ControlPanelConstants.kPIDLoopIdx, ControlPanelConstants.kTimeoutMs);

    /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
    m_motor.config_kF(ControlPanelConstants.kPIDLoopIdx, ControlPanelConstants.kGains.kF,
        ControlPanelConstants.kTimeoutMs);
    m_motor.config_kP(ControlPanelConstants.kPIDLoopIdx, ControlPanelConstants.kGains.kP,
        ControlPanelConstants.kTimeoutMs);
    m_motor.config_kI(ControlPanelConstants.kPIDLoopIdx, ControlPanelConstants.kGains.kI,
        ControlPanelConstants.kTimeoutMs);
    m_motor.config_kD(ControlPanelConstants.kPIDLoopIdx, ControlPanelConstants.kGains.kD,
        ControlPanelConstants.kTimeoutMs);

    /**
     * Grab the 360 degree position of the MagEncoder's absolute position, and
     * intitally set the relative sensor to match.
     */
    int absolutePosition = (int) m_motor.getSensorCollection().getIntegratedSensorAbsolutePosition();

    /* Mask out overflows, keep bottom 12 bits */
    absolutePosition &= 0xFFF;
    if (ControlPanelConstants.kSensorPhase) {
      absolutePosition *= -1;
    }
    if (ControlPanelConstants.kMotorInvert) {
      absolutePosition *= -1;
    }

    /* Set the quadrature (relative) sensor to match absolute */
    m_motor.setSelectedSensorPosition(0, ControlPanelConstants.kPIDLoopIdx,
        ControlPanelConstants.kTimeoutMs);
  }

  public void setSpeed(double speed) {
    m_motor.set(ControlMode.Velocity, speed);
  }

  public void setPosition(double position) {
    m_motor.set(ControlMode.Position, position);
  }

}