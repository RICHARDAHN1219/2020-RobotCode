package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.indexerSubsystem;
import frc.robot.subsystems.shooterSubsystem;

public class shooterHalfCommand extends CommandBase {

  private shooterSubsystem m_shooterSubsystem;
  private indexerSubsystem m_indexer;

  public shooterHalfCommand(shooterSubsystem subsystem, indexerSubsystem indexer) {
    m_shooterSubsystem = subsystem;
    m_indexer = indexer;
    addRequirements(subsystem, indexer);
  }

  @Override
  public void initialize() {
   m_shooterSubsystem.setShooterPID(0.1, 0, 0, 0);
   m_shooterSubsystem.setShooterRPM(3190);
  }

  @Override
  public void execute() {
    m_shooterSubsystem.setShooterRPM(3190);
    //indexerSubsystem.indexKicker.set(ControlMode.PercentOutput, .7);
    m_indexer.setKickerPercentOutput(0.7);
  
  }

  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.setPercentOutput(0.0);
    m_indexer.setKickerPercentOutput(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
