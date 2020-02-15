/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;

/**
 * Field location constants.
 */
public class FieldLocations {

    // coords come from:
    //  https://www.chiefdelphi.com/t/field-coordinates/374470/2
    //  https://github.com/frc1444/robo-sim/blob/master/api/src/main/java/com/first1444/sim/api/frc/implementations/infiniterecharge/Field2020.kt
    //  https://github.com/frc1444/robo-sim/blob/master/gdx/src/main/java/com/first1444/sim/gdx/implementations/infiniterecharge2020/FieldSetup2020.kt

    // TODO: decide where on the field should be the origin (0,0) lower right or middle of field
    public static final double WIDTH = inchesToMeters(26 * 12 + 11.25);
    public static final double LENGTH = inchesToMeters(52 * 12 + 5.25);

    // coordinates of our powerPort
    public static final Translation2d powerPort = new Translation2d(WIDTH / 2 - inchesToMeters(94.66), LENGTH / 2);


   public static double inchesToMeters(double i) {
       return(i * 0.0254);
   }

}
