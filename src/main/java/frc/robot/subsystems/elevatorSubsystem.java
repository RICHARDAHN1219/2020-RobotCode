/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.EncoderType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.elevatorConstants;

public class elevatorSubsystem extends SubsystemBase {

  private Solenoid elevatorDeploySolenoid = new Solenoid(elevatorConstants.solenoid1); // TODO: get correct ids
  private Solenoid brakeSolenoid = new Solenoid(elevatorConstants.brakeSolenoid);
  private CANSparkMax elevatorWinch = new CANSparkMax(elevatorConstants.elevatorWinch, MotorType.kBrushless);
  private final CANEncoder elevatorEncoder = elevatorWinch.getEncoder(EncoderType.kHallSensor, 2048);
  private boolean elevatorDeployed = false;

  /**
   * Creates a new Climber.
   */
  public elevatorSubsystem() {
    elevatorDeploySolenoid.set(false);

    elevatorWinch.restoreFactoryDefaults();
    elevatorWinch.setIdleMode(CANSparkMax.IdleMode.kBrake);
    elevatorEncoder.setInverted(false);
  }

  public void deployElevator() {
    elevatorDeploySolenoid.set(true);
  }

  public void retractElevator() {
    elevatorDeploySolenoid.set(false);
  }

  public void brakeOn() {
    brakeSolenoid.set(true);
  }

  public void brakeOff() {
    brakeSolenoid.set(false);
  }

  public void setWinchPercentOutput(double Percent) {
    elevatorWinch.set(Percent);
  }

  public void setElevatorDeployed(boolean state) {
    elevatorDeployed = state;
  }

  public boolean getElevatorDeployed() {
    return elevatorDeployed;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Winch_RPM", elevatorEncoder.getVelocity());
  }

}
