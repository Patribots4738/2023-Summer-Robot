// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final int DRIVER_CONTROLLER_PORT = 0;

  public static final double DRIVER_DEADBAND = 0.17;

  public final static class DrivetrainConstants {

    public static final int LEFT_MOTOR_FRONT_CAN_ID = 1;
    public static final int LEFT_MOTOR_FOLLOWER_CAN_ID = 2;

    public static final int RIGHT_MOTOR_FRONT_CAN_ID = 3;
    public static final int RIGHT_MOTOR_FOLLOWER_CAN_ID = 4;

    public static final boolean RIGHT_MOTOR_INVERT = false;
    public static final boolean LEFT_MOTOR_INVERT = false;

    public static final double TURNING_P = 0.6;
    public static final double TURNING_I = 0;
    public static final double TURNING_D = 0.1;

    public static final double DRIVING_P = 0.7;
    public static final double DRIVING_I = 0;
    public static final double DRIVING_D = 0.1;

    public static final double TRACK_WIDTH = 0.69;
    public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(TRACK_WIDTH);

    public static final double SLEW_RATE_TURN = .5;
    public static final double SLEW_RATE_DRIVE = .5;
  }
}
