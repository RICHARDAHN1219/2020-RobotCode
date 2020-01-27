package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.indexerSubsystem;
import frc.robot.subsystems.shooterSubsystem;

public class shooterHalfCommand extends CommandBase {

  public final shooterSubsystem m_shooterSubsystem;

  public shooterHalfCommand(shooterSubsystem subsystem) {
    m_shooterSubsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
   m_shooterSubsystem.setShooterPID(0.1, 0, 0, 0);
   m_shooterSubsystem.setShooterRPM(3190);
  }

  @Override
  public void execute() {
    m_shooterSubsystem.setShooterRPM(3190);
    indexerSubsystem.indexKicker.set(ControlMode.PercentOutput, .7);
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.shooter1.set(ControlMode.PercentOutput, 0);
    indexerSubsystem.indexKicker.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
