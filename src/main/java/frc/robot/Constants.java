// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;

import io.github.oblarg.oblog.Loggable;

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

  // This is a list of all of our motors that we use to burn flashes without complication
  public static ArrayList<CANSparkMax> SPARK_LIST = new ArrayList<CANSparkMax>();

  public static final int DRIVER_CONTROLLER_PORT = 0;
  public static final int OPERATOR_CONTROLLER_PORT = 1;

  public static final double DRIVER_DEADBAND_FORWARD = 0.05;
  public static final double DRIVER_DEADBAND_TURN = 0.03;
  public static final double OPERATOR_DEADBAND = 0.5;


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
    public static final double SLEW_RATE_TURN_NEGATIVE = -10;
    public static final double SLEW_RATE_TURN_POSITIVE = 10;
    public static final double SLEW_RATE_DRIVE_POSITIVE = 4;
    public static final double SLEW_RATE_DRIVE_NEGATIVE = -10;
  }

  public final static class PivotConstants implements Loggable{

    public static final double ORIGINAL_ZERO_OFFSET = 16.5016218;
    public static final double ZERO_OFFSET_OFFSET = 180;

    public static final double PIVOT_DEADBAND_DEGREES = 5;

    public static final double PIVOT_LOW_LIMIT_DEGREES = 90 - 20;
    public static final double PIVOT_HIGH_LIMIT_DEGREES = 90 + 155;

    // TODO: change CAN IDs to match the robot
    public static final int PIVOT_LEAD_CAN_ID = 5;
    public static final int PIVOT_FOLLOWER_CAN_ID = 6;

    public static final double PIVOT_P = 0.005;
    public static final double PIVOT_I = 0;
    public static final double PIVOT_D = 0.0525;

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

    // The outtake limit for the claw (its 10 so there is basically nothing limiting it)
    public static final double SLEW_NEGATIVE_LIMIT = -10;
    // The intake limit for the claw (we want to prevent skipping so we have a very low limit)
    public static final double SLEW_POSITIVE_LIMIT = 0.75;

    public static final int CLAW_LEAD_CAN_ID = 8;
    public static final int CLAW_FOLLOWER_CAN_ID = 7;

    public static final int CLAW_FREE_CURRENT_LIMIT = 30;
    public static final int CLAW_STALL_CURRENT_LIMIT = 10;
    public static final int CLAW_OUTTAKE_CURRENT_LIMIT = 30;

  }

  public static final class PlacementConstants {
    public static final int LOW_INDEX = 0;
    public static final int MID_INDEX = 1;
    public static final int HIGH_INDEX = 2;
    public static final int RESET_INDEX = 3;

    // TODO: These values are placeholder positions and need to be tuned for the
    // robot
    public static final double[] PLACEMENT_POSITIONS_FRONT = {
        // Low | Index 0
        90 + 0,
        // Mid | Index 1
        90 + 44,
        // High | Index 2
        90 + 54,
        // Reset | Index 3
        78
    };

    // TODO: These valuse are plceholder speeds and need to be tuned for the robot
    public static final double[] PLACEMENT_SPEEDS_FRONT = {
        // Low | Index 0
        -0.15,
        // Mid | Index 1
        -0.24,
        // High | Index 2
        -0.3,
        // Reset | Index 3
        0.0

    };

    public static final double[] PLACEMENT_POSITIONS_BACK = {
        // Low | Index 0
        270 + 0,
        // Mid | Index 1
        270 - 44,
        // High | Index 2
        270 - 15 - 54,
        // Reset | Index 3
        78
    };

    // TODO: These valuse are plceholder speeds and need to be tuned for the robot
    public static final double[] PLACEMENT_SPEEDS_BACK = {
        // Low | Index 0
        -0.15,
        // Mid | Index 1
        -0.24,
        // High | Index 2
        -0.4,
        // Reset | Index 3
        0.0

    };

    // TODO: Add timings for each position through testing
    // This is an array of times for each placement position
    // If we wanted the outtake time to be different for each position, we could
    public static final double[] PLACEMENT_TIMES = {
        // Low | Index 0
        1, // seconds
        // Mid | Index 1
        1, // seconds
        // High | Index 2
        1, // seconds
        // Reset | Index 3
        0 // seconds
    };
  }
}
