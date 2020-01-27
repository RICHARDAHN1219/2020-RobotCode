/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public final class Constants {
public static final class driveConstants {
    public static final int falcon1 = 0;
    public static final int falcon2 = 1;
    public static final int falcon3 = 2;
    public static final int falcon4 = 3;
    public static final int driveController = 0;
    public static final int operatorController = 1;
}
public static final class turretConstants {
    public static final int turret = 11;
    public static final int kSoftMaxTurretAngle = 1;
    public static final int kSoftMinTurretAngle = 1;
    public static final int kTurretRotationsPerTick = 1;
}
public static final class shooterConstants {
    public static final int shooter1 = 4;
    public static final int shooter2 = 5;
    public static final int shooterTimeout = 30;
    public static final int shooterSlotIdx = 0;
}
public static final class elevatorConstants {
    public static final int elevator1 = 6;
    public static final int elevator2 = 7;
    public static final int elevatorWinch = 12;
    public static final int elevatorPivotTimeout = 30;
    public static final int elevatorSlotIdx = 1;
}
public static final class indexConstants {
    public static final int index1_1 = 8;
    public static final int index1_2 = 9;
    public static final int indexKicker = 10;
}
public static final class controlPanelConstants {
    public static final int motor = 4;
    public static final int controlPanelSlotIdx = 0;
    public static final int PIDLoopIdx = 0;
    public static final int timeoutMs = 30;
    public static boolean sensorPhase = true;
    public static boolean motorInvert = false;
    public static final Gains gains = new Gains(0.15, 0.0, 0.0, 0.0, 0, 1.0);
}
public static final class intakeConstants {
    public static final int intakeMotor = 20;
}
}