/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

/**
 * Field location constants.
 */
public class FieldLocations {

    // coords come from:
    // https://www.chiefdelphi.com/t/field-coordinates/374470/2
    // https://github.com/frc1444/robo-sim/blob/master/api/src/main/java/com/first1444/sim/api/frc/implementations/infiniterecharge/Field2020.kt
    // https://github.com/frc1444/robo-sim/blob/master/gdx/src/main/java/com/first1444/sim/gdx/implementations/infiniterecharge2020/FieldSetup2020.kt

    // Notes on field coordinates.
    //
    // Origin (0,0) is Left, Back of the Field from the driver station's perspective
    // All X values will be positive
    // All Y values will be negative
    // Pose: 0 degrees is robot pointed away from the driver station
    // 90 degrees is robot pointed towards left side of field
    // 180 degrees is robot pointed towards the driver station side
    // -90 degrees is robot pointed towards right side if field


    public static final double WIDTH = inchesToMeters(26 * 12 + 11.25);
    // 5.26 inches comes from page 5 of
    // https://firstfrc.blob.core.windows.net/frc2020/PlayingField/LayoutandMarkingDiagram.pdf
    public static final double LENGTH = inchesToMeters(52 * 12 + 5.26); // or about 629.5

    // X coords of initiation lines
    public static final double INITIATION_LINE_X = LENGTH - inchesToMeters(120);
    public static final double ENEMY_INITIATION_LINE_X = inchesToMeters(120);

    // General form: WIDTH / 2 - inchesToMeters(distance from right guardrail (relative to our Power
    // Port) to point on field of coordinate), LENGTH / 2 - inchesToMeters(distance from our
    // alliance's Alliance Station 2 to point on field of coordinate)
    // coordinates of our Power Port
    public static final Translation2d powerPort =
            new Translation2d(LENGTH, -WIDTH + inchesToMeters(94.66));

    // coordinates of left Power Cell in enemy trench
    public static final Translation2d powerCell1 =
            new Translation2d(inchesToMeters(120 + 258.90), -inchesToMeters(27.75 - 10.0));
    // coordinates of 2nd leftmost Power Cell in enemy trech
    public static final Translation2d powerCell2 =
            new Translation2d(inchesToMeters(120 + 258.90), -inchesToMeters(27.75 + 10.0));

    // coordinates of 7th extra Power Cell relative to our Power Port
    // public static final Translation2d extraPowerCell7 = new Translation2d(WIDTH / 2 -
    // inchesToMeters(), LENGTH / 2 - inchesToMeters());


    // Coords for Auton 1
    public static final Pose2d AutonPath1_1_StartPose = 
        new Pose2d(new Translation2d(INITIATION_LINE_X, -1.837538, new Rotation2d(Math.PI));
    public static final Pose2d AutonPath1_1_EndPose = 
        new Pose2d(new Translation2d(9.6943852, -0.77532), new Rotation2d(1.89254));
    public static final List<Translation2d> AutonPath1_1_IntermediatePoints = 
        List.of(
            new Translation2d(11.504489689611543, -1.7226037844366988)
        );

    
    public static final Pose2d AutonPath1_2_StartPose = 
        new Pose2d(new Translation2d(9.6943852, -0.77532), new Rotation2d(1.89254));
    public static final Pose2d AutonPath1_2_EndPose = 
        new Pose2d(new Translation2d(9.6943852, -0.77532), new Rotation2d(1.89254));
    public static final List<Translation2d> AutonPath1_2_IntermediatePoints = 
        List.of(
        // Empty list    
        );
    


    public static double inchesToMeters(double i) {
        return (i * 0.0254);
    }

}
