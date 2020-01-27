/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.indexerSubsystem;
import frc.robot.subsystems.shooterSubsystem;

public class shooterCommand extends CommandBase {
  
  public final shooterSubsystem m_shooterSubsystem;

  public shooterCommand(shooterSubsystem subsystem) {
    m_shooterSubsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_shooterSubsystem.setShooterPID(0.1, 0, 0, 0);
    //m_shooterSubsystem.setShooterRPM(6380);
  }

  @Override
  public void execute() {
    System.out.println("Spooling Flywheel");
    /*
    double ta = NetworkTableInstance.getDefault().getTable("limelight-one").getEntry("ta").getDouble(0);
    if (ta == 0) {
      shooterSubsystem.shooter1.set(ControlMode.PercentOutput, 0);
    } 
    else {
      shooterSubsystem.shooter1.set(ControlMode.PercentOutput, 1-(100/ta));
    }
    System.out.println(1*100/ta);
    */
    
    shooterSubsystem.shooter1.set(ControlMode.Position, 2048);
    new WaitCommand(0.5);
    //Use 1500 for testing purposes, as it is exactly half speed.
    //m_shooterSubsystem.setShooterRPM(1500);
    System.out.println("Starting Kicker Wheel");
    //IndexerSubsystem.indexLoad.set(ControlMode.PercentOutput, 1);
  }
  
  @Override
  public void end(boolean interrupted) {
    System.out.println("Stopping Flywheel/Kicker Wheel");
    shooterSubsystem.shooter1.set(ControlMode.PercentOutput, 0);
    indexerSubsystem.indexKicker.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
