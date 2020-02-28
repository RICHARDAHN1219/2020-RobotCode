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

public class hoodDownAutoShootCommand extends CommandBase {

  indexerSubsystem m_indexer;
  turretSubsystem m_turret;
  shooterSubsystem m_shooter;
  limelight m_limelight;

  private double steer_k = 0.1;
  private double tv;
  private double tx;
  private double limelightSteerCommand = 0;

  public hoodDownAutoShootCommand(indexerSubsystem indexer, turretSubsystem turret, shooterSubsystem shooter, limelight ll_util) {
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
    m_limelight.setLEDMode(0);
    m_shooter.retractHood();
  }

  @Override
  public void execute() {
    tv = m_limelight.getTV();
    tx = m_limelight.getTX();

    if (tv != 1) {
      RobotContainer.limelightOnTarget = false;
      limelightSteerCommand = 0;
      m_turret.setPercentOutput(RobotContainer.m_operatorController.getX(Hand.kLeft) * 0.5);
      return;
    }
    
    m_shooter.setShooterRPM(m_shooter.getRPMforTY(m_limelight.getTY()));

    limelightSteerCommand = tx * steer_k;
    m_turret.setPercentOutput(limelightSteerCommand);

    if (Math.abs(m_limelight.getTX()) < 0.25) {
      RobotContainer.limelightOnTarget = true;
    }
    else {
      RobotContainer.limelightOnTarget = false;
    }

    if (m_shooter.isAtSpeed() == true && RobotContainer.limelightOnTarget == true) {
      m_indexer.ejectOneBall();
    }
  }
  
  @Override
  public void end(boolean interrupted) {
    m_indexer.stopIndexer();
    m_shooter.setShooterRPM(0);
    m_turret.stop();
    RobotContainer.limelightOnTarget = false;
    m_limelight.setLEDMode(1);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
