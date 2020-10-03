/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.fearxzombie.limelight;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.indexerSubsystem;
import frc.robot.subsystems.shooterSubsystem;
import frc.robot.subsystems.turretSubsystem;

public class shootOneBallCommand extends CommandBase {

  indexerSubsystem m_indexer;
  turretSubsystem m_turret;
  shooterSubsystem m_shooter;
  limelight m_limelight;

  private double steer_k = 0.1;
  private double tv;
  private double tx;
  private double limelightSteerCommand = 0;
  private boolean finished;
  
  public shootOneBallCommand(indexerSubsystem indexer, turretSubsystem turret, shooterSubsystem shooter, limelight ll_util) {
    addRequirements(indexer);
    addRequirements(turret);
    addRequirements(shooter);
    m_indexer = indexer;
    m_turret = turret;
    m_shooter = shooter;
    m_limelight = ll_util;
  }

  @Override
  public void initialize() {
    finished = false;
  }

  @Override
  public void execute() {
    m_shooter.setShooterRPM(m_shooter.getRPMforTY(m_limelight.getTY()));
    tv = m_limelight.getTV();
    tx = m_limelight.getTX();

    if (tv != 1) {
      limelightSteerCommand = 0;
      m_turret.setPercentOutput(RobotContainer.m_operatorController.getX(Hand.kLeft));
      return;
    }

    limelightSteerCommand = tx * steer_k;
    m_turret.setPercentOutput(limelightSteerCommand);

    if (tv == 1 && m_shooter.isAtSpeed() == true && Math.abs(tx) < 2.5) {
      // TODO: this is not the correct way to start a new command, need make a command group
      // It's the right idea, and needs a command group to schedule one thing after another
      // I see this command isn't used. Perhapse it's best to delete this file to avoid confusion?
      new indexerSingleFeedCommand(m_indexer);
      finished = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_indexer.stopIndexer();
    m_shooter.stop();
    m_turret.stop();
  }

  @Override
  public boolean isFinished() {
    if (m_indexer.getFinishedSingleFeed() == true && finished == true) {
      return true;
    }

    return false;
  }
}
