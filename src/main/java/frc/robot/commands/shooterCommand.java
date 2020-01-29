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
  
  private shooterSubsystem m_shooterSubsystem;
  private indexerSubsystem m_indexer;

  public shooterCommand(shooterSubsystem subsystem, indexerSubsystem indexer) {
    m_shooterSubsystem = subsystem;
    m_indexer = indexer;
    addRequirements(subsystem, indexer);
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
    
    //m_shooterSubsystem.setPosition(2048);    // TODO: This needs to be a speed not a position
    //new WaitCommand(0.5);
    //Use 1500 for testing purposes, as it is exactly half speed.
    //m_shooterSubsystem.setShooterRPM(1500);
    // System.out.println("Starting Kicker Wheel");
    //IndexerSubsystem.indexLoad.set(ControlMode.PercentOutput, 1);
  }
  
  @Override
  public void end(boolean interrupted) {
    System.out.println("Stopping Flywheel/Kicker Wheel");
    m_shooterSubsystem.setPercentOutput(0.0);
    m_indexer.setKickerPercentOutput(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
