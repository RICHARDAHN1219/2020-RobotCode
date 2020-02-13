/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.limelightTurretVisionCommand;

public class limelightSubsystem extends SubsystemBase {
  /**
   * Creates a new limelightSubsystem.
   */
  private String limelightName;
  NetworkTableInstance getNT = NetworkTableInstance.getDefault();
  private NetworkTable limelightNT;
  public limelightSubsystem(String limelightName) {
    limelightNT = getNT.getTable(limelightName);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  /**
 * set1xZoom() - Sets zoom level to 1x HW Zoom on Limelight.
 * @return void
 */
public void set1xZoom(){
  limelightNT.getEntry("pipeline").setNumber(0);
}
/**
 * set2xZoom() - Sets zoom level to 2x HW Zoom on Limelight.
 * @return void.
 */
public void set2xZoom(){
  limelightNT.getEntry("pipeline").setNumber(1);
}
/**
 * set3xZoom() - Sets zoom level to 3x HW Zoom on Limelight.
 * @return void
 */
public void set3xZoom(){
  limelightNT.getEntry("pipeline").setNumber(2);
}
/**
 * setLEDMode() - Sets LED mode.
 * @return void
 */
public void setLEDMode(double value) {
  limelightNT.getEntry("ledMode").setNumber(value);
}
/**
 * setCAMMode() - Sets camera mode.
 * @return void
 */
public void setCAMMode(double value) {
  limelightNT.getEntry("camMode").setNumber(value);
}
/**
 * getTV() - monitor the Limelight's TV Value
 * @return 1 if limelight target lock, 0 if no lock.
 */
public double getTV(){
  return limelightNT.getEntry("tv").getDouble(0);
}
/**
 * getTA() - monitor the Limelight's TA Value
 * @return Area of image taken up by target. 0%-100% range.
 */
public double getTA(){
  return limelightNT.getEntry("ta").getDouble(0);
}
/**
 * getTX() - monitor the Limelight's TX Value
 * @return Horizontal Offset From Crosshair To Target (-29.8 to 29.8 degrees)
 */
public double getTX(){
  return limelightNT.getEntry("tx").getDouble(0);
}
/**
 * getTY() - monitor the Limelight's TY Value
 * @return Vertical Offset From Crosshair To Target (-24.85 to 24.85 degrees)
 */
public double getTY(){
  return limelightNT.getEntry("ty").getDouble(0);
}
/**
 * getPipeline() - monitor the Limelight's pipeline Value
 * @return pipeline
 */
public double getPipeline(){
  return limelightNT.getEntry("pipeline").getDouble(0);
}
/**
 * get() - monitor any value needed outside of currently provided.
 * @return value of key
 */
public double get(String entry){
  return limelightNT.getEntry(entry).getDouble(0);
}
}