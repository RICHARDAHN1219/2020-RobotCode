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

public class limelightSubsystem extends SubsystemBase {
  /**
   * Creates a new limelightSubsystem.
   */
  private String limelightName;
  NetworkTableInstance getNT = NetworkTableInstance.getDefault();
  private NetworkTable limelightNT;

  public limelightSubsystem() {
    limelightName = "limelight-one";
    limelightNT = getNT.getTable(limelightName);
  }

  public limelightSubsystem(String name) {
    limelightName = name;
    limelightNT = getNT.getTable(limelightName);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
        //Example output: currentDist = (98.5-24 / tan(10+20))
    //currentDist = 129.03 so 3x Zoom shall be used.
    double h1 = 24;
    double h2 = 98.5;
    double a1 = 10;
    double a2 = getTY();
    double oneXDist = 68;
    double twoXDist = 107;
    double threeXDist = 117;
    double currentDist = Math.abs(h2 - h1) / Math.tan(a1 + a2);
    if (currentDist >= oneXDist){
      set1xZoom();
      System.out.println("Switching to 1x Zoom");
    } else if (currentDist >= twoXDist){
      set2xZoom();
      System.out.println("Switching to 2x Zoom");
    } else if (currentDist >= threeXDist) {
      set3xZoom();
      System.out.println("Switching to 3x Zoom");
    }
  }

  /**
   * set1xZoom() - Sets zoom level to 1x HW Zoom on Limelight.
   * 
   * @return void
   */
  public void set1xZoom() {
    limelightNT.getEntry("pipeline").setNumber(0);
  }

  /**
   * set2xZoom() - Sets zoom level to 2x HW Zoom on Limelight.
   * 
   * @return void.
   */
  public void set2xZoom() {
    limelightNT.getEntry("pipeline").setNumber(1);
  }

  /**
   * set3xZoom() - Sets zoom level to 3x HW Zoom on Limelight.
   * 
   * @return void
   */
  public void set3xZoom() {
    limelightNT.getEntry("pipeline").setNumber(2);
  }

  /**
   * setLEDMode() - Sets LED mode. 0 use the LED Mode set in the current pipeline, 1 force off, 2
   * force blink, 3 force on
   * 
   * @return void
   */
  public void setLEDMode(double value) {
    limelightNT.getEntry("ledMode").setNumber(value);
  }

  /**
   * setCAMMode() - Sets camera mode. 0 for Vision processor, 1 for Driver Camera (Increases
   * exposure, disables vision processing)
   * 
   * @return void
   */
  public void setCAMMode(double value) {
    limelightNT.getEntry("camMode").setNumber(value);
  }

  /**
   * setStreamMode() - Sets limelight’s streaming mode -0 Standard - Side-by-side streams if a
   * webcam is attached to Limelight - 1 PiP Main - The secondary camera stream is placed in the
   * lower-right corner of the primary camera stream - 2 PiP Secondary - The primary camera stream
   * is placed in the lower-right corner of the secondary camera stream
   * 
   * @return void
   */
  public void setStreamMode(double value) {
    limelightNT.getEntry("stream").setNumber(value);
  }

  /**
   * getTV() - Monitor whether the limelight has any valid targets (0 or 1)
   * 
   * @return 1 if limelight target lock, 0 if no lock.
   */
  public double getTV() {
    return limelightNT.getEntry("tv").getDouble(0);
  }

  /**
   * getTA() - Target Area (0% of image to 100% of image)
   * 
   * @return (0-100)
   */
  public double getTA() {
    return limelightNT.getEntry("ta").getDouble(0);
  }

  /**
   * getTX() - monitor the Limelight's TX Value
   * 
   * @return Horizontal Offset From Crosshair To Target (-29.8 to 29.8 degrees)
   */
  public double getTX() {
    return limelightNT.getEntry("tx").getDouble(0);
  }

  /**
   * getTY() - monitor the Limelight's TY Value
   * 
   * @return Vertical Offset From Crosshair To Target (-24.85 to 24.85 degrees)
   */
  public double getTY() {
    return limelightNT.getEntry("ty").getDouble(0);
  }

  /**
   * getPipeline() - monitor the Limelight's pipeline Value
   * 
   * @return pipeline
   */
  public double getPipeline() {
    return limelightNT.getEntry("pipeline").getDouble(0);
  }

  /**
   * getCameraTranslation() - Results of a 3D position solution
   * 
   * @return 6 numbers: Translation (x,y,y) Rotation(pitch,yaw,roll)
   */
  public double getCameraTranslation() {
    return limelightNT.getEntry("camtran").getDouble(0);
  }

  /**
   * get() - monitor any value needed outside of currently provided.
   * 
   * @return value of key
   */
  public double get(String entry) {
    return limelightNT.getEntry(entry).getDouble(0);
  }
}
