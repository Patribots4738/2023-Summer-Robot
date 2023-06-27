// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static int leftMotorFront = 1;
    public static int leftFollower = 2;

    public static int rightMotorFront = 3;
    public static int rightFollower = 4;

    public static boolean rightMotorInvert = true;
    public static boolean leftMotorInvert = true;

    public static int driverControllerPort = 0;

    public static double DRIVER_DEADBAND = 0.17;
}
