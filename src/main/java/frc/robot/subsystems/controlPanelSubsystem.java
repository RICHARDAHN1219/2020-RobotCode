/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.controlPanelConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class controlPanelSubsystem extends SubsystemBase {
  // space for variables
  private WPI_TalonSRX controlPanelMotor = new WPI_TalonSRX(controlPanelConstants.motor);
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  String lastSeenColor = "Unknown";
  String gameData;
  private int count = 0;
  /*
   * Color Wheel Blue CMY: 100,0,0 RGB: #00FFFF Green CMY: 100,0,100 RGB: #00FF00
   * Red CMY: 0,100,100 RGB: #FF0000 Yellow CMY: 0,0,100 RGB: #FFFF00
   */
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  public controlPanelSubsystem() {
    //Restoring the motors to default settings
    controlPanelMotor.configFactoryDefault();
    controlPanelMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, controlPanelConstants.PIDLoopIdx, controlPanelConstants.timeoutMs);
    controlPanelMotor.setSensorPhase(controlPanelConstants.sensorPhase);
    controlPanelMotor.setInverted(controlPanelConstants.motorInvert);
    controlPanelMotor.configNominalOutputForward(0, controlPanelConstants.timeoutMs);
    controlPanelMotor.configNominalOutputReverse(0, controlPanelConstants.timeoutMs);
    controlPanelMotor.configPeakOutputForward(.2, controlPanelConstants.timeoutMs);
    controlPanelMotor.configPeakOutputReverse(-.2, controlPanelConstants.timeoutMs);
    controlPanelMotor.configAllowableClosedloopError(0, controlPanelConstants.PIDLoopIdx, controlPanelConstants.timeoutMs);
    controlPanelMotor.config_kF(controlPanelConstants.PIDLoopIdx, controlPanelConstants.gains.kF, controlPanelConstants.timeoutMs);
    controlPanelMotor.config_kP(controlPanelConstants.PIDLoopIdx, controlPanelConstants.gains.kP, controlPanelConstants.timeoutMs);
    controlPanelMotor.config_kI(controlPanelConstants.PIDLoopIdx, controlPanelConstants.gains.kI, controlPanelConstants.timeoutMs);
    controlPanelMotor.config_kD(controlPanelConstants.PIDLoopIdx, controlPanelConstants.gains.kD, controlPanelConstants.timeoutMs);
    int absolutePosition = controlPanelMotor.getSelectedSensorPosition();
    absolutePosition &= 0xFFF;
    
    if (controlPanelConstants.sensorPhase) {
      absolutePosition *= -1;
    }
    
    if (controlPanelConstants.motorInvert) {
      absolutePosition *= -1;
    }

    controlPanelMotor.setSelectedSensorPosition(0, controlPanelConstants.PIDLoopIdx, controlPanelConstants.timeoutMs);
  }

  public void setSpeed(double speed) {
    controlPanelMotor.set(ControlMode.Velocity, speed);
    // colors we want to match
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);
  }

  public void senseColorWheelPos() {

    String colorString = getColor();

    int proximity = m_colorSensor.getProximity();
    SmartDashboard.putNumber("Proximity", proximity);

    /* Code for counting colors */

    if (lastSeenColor.equals("Red")) {
      if (colorString.equals("Green")) {
        count = count + 1;
      }
      if (colorString.equals("Yellow")) {
        count = count - 1;
      }
    } else if (lastSeenColor.equals("Green")) {
      if (colorString.equals("Blue")) {
        count = count + 1;
      }
      if (colorString.equals("Red")) {
        count = count - 1;
      }
    } else if (lastSeenColor.equals("Blue")) {
      if (colorString.equals("Yellow")) {
        count = count + 1;
      }
      if (colorString.equals("Green")) {
        count = count - 1;
      }
    } else if (lastSeenColor.equals("Yellow")) {
      if (colorString.equals("Red")) {
        count = count + 1;
      }
      if (colorString.equals("Blue")) {
        count = count - 1;
      }
    }
    // Color reset and count display on SmartDashboard
    lastSeenColor = colorString;
    SmartDashboard.putNumber("Count", count);

    // TODO: Detect errors and unknown colors
  }

  public String getColor() {
    // This method will be called once per scheduler run
    Color detectedColor = m_colorSensor.getColor();

    /**
     * Run the color match algorithm on our detected color
     */
    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);

    return colorString;
  }

  public int colorNumbers() {
    String currentColor = getColor();
    char currentColorChar = currentColor.charAt(0);
    char stage2ColorChar = gameData.charAt(0);

    if (currentColorChar == 'B') {
      return 0;
    }
    if (stage2ColorChar == 'B') {
      return 0;
    }
    if (currentColorChar == 'Y') {
      return 1;
    }
    if (stage2ColorChar == 'Y') {
      return 1;
    }
    if (currentColorChar == 'R') {
      return 2;
    }
    if (stage2ColorChar == 'R') {
      return 2;
    }
    if (currentColorChar == 'G') {
      return 3;
    }
    if (stage2ColorChar == 'G') {
      return 3;
    }
    return ' ';
  }

  public void moveToGamePosition(char targetColorChar) {

    String currentColor = getColor();
    char currentColorChar = currentColor.charAt(0);
    String gameData = DriverStation.getInstance().getGameSpecificMessage();
    char stage2ColorChar = gameData.charAt(0);

    /*Beginnings of new way, needs to be reviewed still once it can be figured out 
    how to move the motors the amount of one color panel*/
    if (stage2ColorChar == 'R' && currentColorChar == 'R') {
      // move either clockwise 6 or counterclockwise 2 (game sees b, move 2 to r)
      // isFinished when currentColorChar == 'B'
    }
    if (stage2ColorChar == 'R' && currentColorChar == 'Y') {
      // move counterclockwise 1 (game sees g, move to r)
      // isFinished when currentColorChar == 'B'
    }
    if (stage2ColorChar == 'R' && currentColorChar == 'G') {
      // move clockwise 1 (game sees g, move to r)
      // isFinished when currentColorChar == 'B'
    }
    if (stage2ColorChar == 'R' && currentColorChar == 'B') {
      // don't move
    }

    /* 
    Old way that needs to be revised to new way

    if (gameData.equals("R") && lastSeenColor.equals("Yellow")) {
      // move counterclockwise until we see blue
    }
    if (gameData.equals("R") && lastSeenColor.equals("Green")) {
      // move clockwise 1
    }
    if (gameData.equals("B") && lastSeenColor.equals("Blue")) {
      // stay at color
    }
    if (gameData.equals("B") && lastSeenColor.equals("Green")) {
      // move counterclockwise 1
    }
    if (gameData.equals("B") && lastSeenColor.equals("Yellow")) {
      // move clockwise 1
    }
    */

  }

  public void setPosition(double position) {
    controlPanelMotor.set(ControlMode.Position, position);
  }

  public double getRotationCount() {
    return (count / 8);
  }
}
