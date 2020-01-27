/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.controlPanelConstants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class controlPanelSubsystem extends SubsystemBase {

  private WPI_TalonSRX colorWheelMotor = new WPI_TalonSRX(controlPanelConstants.motor);

  public controlPanelSubsystem() {

    colorWheelMotor.configFactoryDefault();
    colorWheelMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, controlPanelConstants.PIDLoopIdx, controlPanelConstants.timeoutMs);
    colorWheelMotor.setSensorPhase(controlPanelConstants.sensorPhase);
    colorWheelMotor.setInverted(controlPanelConstants.motorInvert);
    colorWheelMotor.configNominalOutputForward(0, controlPanelConstants.timeoutMs);
    colorWheelMotor.configNominalOutputReverse(0, controlPanelConstants.timeoutMs);
    colorWheelMotor.configPeakOutputForward(.2, controlPanelConstants.timeoutMs);
    colorWheelMotor.configPeakOutputReverse(-.2, controlPanelConstants.timeoutMs);
    colorWheelMotor.configAllowableClosedloopError(0, controlPanelConstants.PIDLoopIdx, controlPanelConstants.timeoutMs);
    colorWheelMotor.config_kF(controlPanelConstants.PIDLoopIdx, controlPanelConstants.gains.kF, controlPanelConstants.timeoutMs);
    colorWheelMotor.config_kP(controlPanelConstants.PIDLoopIdx, controlPanelConstants.gains.kP, controlPanelConstants.timeoutMs);
    colorWheelMotor.config_kI(controlPanelConstants.PIDLoopIdx, controlPanelConstants.gains.kI, controlPanelConstants.timeoutMs);
    colorWheelMotor.config_kD(controlPanelConstants.PIDLoopIdx, controlPanelConstants.gains.kD, controlPanelConstants.timeoutMs);
    int absolutePosition = colorWheelMotor.getSelectedSensorPosition();
    absolutePosition &= 0xFFF;
    
    if (controlPanelConstants.sensorPhase) {
      absolutePosition *= -1;
    }
    
    if (controlPanelConstants.motorInvert) {
      absolutePosition *= -1;
    }

    colorWheelMotor.setSelectedSensorPosition(0, controlPanelConstants.PIDLoopIdx, controlPanelConstants.timeoutMs);
  }

  public void setSpeed(double speed) {
    colorWheelMotor.set(ControlMode.Velocity, speed);
  }

  public void setPosition(double position) {
    colorWheelMotor.set(ControlMode.Position, position);
  }
}