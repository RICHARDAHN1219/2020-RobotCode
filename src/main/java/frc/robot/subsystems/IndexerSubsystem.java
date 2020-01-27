/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.indexConstants;
import edu.wpi.first.wpilibj.DigitalInput;

public class indexerSubsystem extends SubsystemBase {

  public static WPI_TalonFX indexStage1_1 = new WPI_TalonFX(indexConstants.index1_1);
  public static WPI_TalonFX indexStage1_2 = new WPI_TalonFX(indexConstants.index1_2);
  public static WPI_TalonFX indexKicker = new WPI_TalonFX(indexConstants.indexKicker);
  public static DigitalInput Sensor1 = new DigitalInput(0);
  public static DigitalInput Sensor2 = new DigitalInput(1);
  public static DigitalInput Sensor3 = new DigitalInput(2);

  public indexerSubsystem() {
    indexStage1_2.follow(indexStage1_1);
    indexStage1_1.setInverted(true);
  }

  @Override
  public void periodic() {
    if (indexStage1_1.getSupplyCurrent() <= 20 || indexStage1_2.getSupplyCurrent() <= 20){
      indexStage1_1.set(ControlMode.PercentOutput, -0.5);
    }
  }
}
