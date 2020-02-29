/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.indexerSubsystem;
import frc.robot.subsystems.intakeSubsystem;

public class indexerDefaultCommand extends CommandBase {

  private indexerSubsystem m_indexer;
  private intakeSubsystem m_intake;
  private boolean clearedSensor2 = false;
  private XboxController opController = RobotContainer.m_operatorController;

  public indexerDefaultCommand(indexerSubsystem indexer, intakeSubsystem intake) {
    addRequirements(indexer);
    addRequirements(intake);
    m_indexer = indexer;
    m_intake = intake;
  }

  @Override
  public void initialize() {
    clearedSensor2 = false;
  }

  @Override
  public void execute() {

    // TODO: tune the speeds of these to not destory balls
    if (opController.getTriggerAxis(Hand.kRight) >= 0.1) {
      m_indexer.setIntakePercentOutput(opController.getTriggerAxis(Hand.kRight));
      m_indexer.setBeltsPercentOutput(opController.getTriggerAxis(Hand.kRight));
      m_indexer.setKickerPercentOutput(opController.getTriggerAxis(Hand.kRight));
    }

    // TODO: tune the speeds of these to not destory balls
    else if (opController.getTriggerAxis(Hand.kLeft) >= 0.1) {
      m_indexer.setIntakePercentOutput(-opController.getTriggerAxis(Hand.kLeft));
      m_indexer.setBeltsPercentOutput(-opController.getTriggerAxis(Hand.kLeft));
      m_indexer.setKickerPercentOutput(-opController.getTriggerAxis(Hand.kLeft));
    }

    else {
      if (opController.getAButton() == true) {
        
        m_intake.deployIntake();
        m_indexer.setIntakePercentOutput(1);
      }
      else {
        m_intake.retractIntake();
        m_indexer.stopIntake();
      }

      if (m_indexer.ballStaged() == false) {
        clearedSensor2 = true;
      }
        
      if (m_indexer.ballExiting() == true) {
        // Always stop indexer if a ball is at the exit point.
        // Don't eject a ball unless we're shooting.
        m_indexer.stopIndexer();
      }
      else if (m_indexer.ballReadyForIndexer() == true) {
        // here we know ballExiting() == false
        // OK to pull in more balls and continue to fill indexer
        m_indexer.setBeltsRPM(6380);
        m_indexer.setKickerPercentOutput(0.3);
      }
    } 
  }

  @Override
  public void end(boolean interrupted) {
    m_indexer.stopIndexer();
  }

  @Override
  public boolean isFinished() {
    if (clearedSensor2 && m_indexer.ballStaged()) {
      return true;
    }
    return false;
  }
}
