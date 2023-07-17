// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
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

    // TODO: These values are placeholders and need to be tuned for the robot
    public static final double SLEW_RATE_TURN = 0;
    public static final double SLEW_RATE_DRIVE = 0;
  }

  public final static class PivotConstants {

    public static final double PIVOT_DEADBAND_DEGREES = 5;

    // TODO: change CAN IDs to match the robot
    public static final int PIVOT_LEAD_CAN_ID = 5;
    public static final int PIVOT_FOLLOWER_CAN_ID = 6;

    public static final double PIVOT_P = 0.1;
    public static final double PIVOT_I = 0;
    public static final double PIVOT_D = 0.1;

    public static final int LOW_INDEX = 0;
    public static final int MID_INDEX = 1;
    public static final int HIGH_INDEX = 2;
    public static final int RESET_INDEX = 3;

    // Multiply all encoder outputs by this factor to get degrees
    public static final int PIVOT_POSITION_ENCODER_FACTOR = 360;
    public static final int PIVOT_SMART_CURRENT_LIMIT = 30;
  }

  public final static class ClawConstants {
    // TODO: change CAN IDs to match the robot
    public static final int CLAW_LEAD_CAN_ID = 5;
    public static final int CLAW_FOLLOWER_CAN_ID = 6;

    public static final double CLAW_P = 0.1;
    public static final double CLAW_I = 0;
    public static final double CLAW_D = 0.1;

    public static final int CLAW_FREE_CURRENT_LIMIT = 30;
    public static final int CLAW_STALL_CURRENT_LIMIT = 10;

  }

  public static final class PlacementConstants {
    public static final int LOW_INDEX = 0;
    public static final int MID_INDEX = 1;
    public static final int HIGH_INDEX = 2;
    public static final int RESET_INDEX = 3;

    // TODO: These values are placeholder positions and need to be tuned for the robot
    public static final double[] PLACEMENT_POSITIONS = {
        // Low | Index 0
        10,
        // Mid | Index 1
        50,
        // High | Index 2
        70,
        // Reset | Index 3
        0
    };

    // TODO: These valuse are plceholder speeds and need to be tuned for the robot
    public static final double[] PLACEMENT_SPEEDS = {
        // High | Index 0
        0.7,
        // Mid | Index 1
        0.5,
        // Low | Index 2
        0.3,
        // Stop | Index 3
        0.0
    };

    // TODO: Add timings for each position through testing
    // This is an array of times for each placement position
    // If we wanted the outtake time to be different for each position, we could
		public static final double[] PLACEMENT_TIMES = {
        // High | Index 0
        1, //seconds 
        // Mid | Index 1
        1, //seconds 
        // Low | Index 2
        1, //seconds 
        // Stop | Index 3
        1  //seconds 
    };
  }
}
