/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.elevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class elevatorRetract extends SequentialCommandGroup {
  /**
   * Creates a new retractDeploy.
   */
  public elevatorRetract(elevatorSubsystem elevator) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    // super()
    addCommands(new InstantCommand(elevator::retract2, elevator), 
                new WaitCommand(0.5),
                new InstantCommand(elevator::retract1, elevator));
  }
}
