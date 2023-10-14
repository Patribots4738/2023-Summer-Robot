// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
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

    // This is a list of all of our motors that we use to burn flashes without
    // complication
    public static ArrayList<CANSparkMax> SPARK_LIST = new ArrayList<CANSparkMax>();

    public final static class ControllerConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;

        public static final double DRIVER_DEADBAND_FORWARD = 0.05;
        public static final double DRIVER_DEADBAND_TURN = 0.03;

        public static final double BETA = 2;
        public static final double ZETA = 0.7;

        public static final double RAMSETE_KP = 1;
        public static final double RAMSETE_KI = 0;
        public static final double REMSETE_KD = 0;
    }

    public final static class DrivetrainConstants {

        public static final double WHEEL_DIAMETER_METERS = 0.15;
        public static final double WHEEL_CIRC_METERS = WHEEL_DIAMETER_METERS * Math.PI;
        public static final double DRIVETRAIN_GEAR_RATIO = 5.95;
        public static final double DRIVING_ENCODER_POS_FACTOR = WHEEL_CIRC_METERS / DRIVETRAIN_GEAR_RATIO; // meters
        public static final double DRIVING_ENCODER_VEL_FACTOR = DRIVING_ENCODER_POS_FACTOR / 60; // m/s

        public static final double DRIVING_SPEED_MULTIPLIER = 1;

        public static final int LEFT_MOTOR_FRONT_CAN_ID = 1;
        public static final int LEFT_MOTOR_FOLLOWER_CAN_ID = 2;

        public static final int RIGHT_MOTOR_FRONT_CAN_ID = 3;
        public static final int RIGHT_MOTOR_FOLLOWER_CAN_ID = 4;

        public static final boolean RIGHT_MOTOR_INVERT = false;
        public static final boolean LEFT_MOTOR_INVERT = false;

        // TODO: Use these values
        public static final double TURNING_P = 0.6;
        public static final double TURNING_I = 0;
        public static final double TURNING_D = 0.1;

        public static final double DRIVING_P = 0.7;
        public static final double DRIVING_I = 0;
        public static final double DRIVING_D = 0.1;

        public static final double SLEW_RATE_TURN_NEGATIVE = -5;
        public static final double SLEW_RATE_TURN_POSITIVE = 5;
        public static final double SLEW_RATE_DRIVE_POSITIVE = 5;
        public static final double SLEW_RATE_DRIVE_NEGATIVE = -5;

        // TODO: Tune these values
        public static final int DRIVE_TO_DISTANCE_TOLERANCE = 4;
        public static final double ANGLE_TOLERANCE = 0.1;

        private static final double TRACK_WIDTH_METERS = 0.5588;

        public static final double MAX_DRIVE_VELOCITY = 4;
        public static final double MAX_DRIVE_ACCELERATION = 1.5;

        public static final double MAX_DRIVE_VOLTAGE = 7;

        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;

        public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(
                DrivetrainConstants.TRACK_WIDTH_METERS);

        public static final double MAX_DRIVE_SPEED = 0.8;
    }

    public final static class PivotConstants implements Loggable {

        // TODO: Use these values
        public static final double ORIGINAL_ZERO_OFFSET = 16.5016218;
        // TODO: Use these values
        public static final double ZERO_OFFSET_OFFSET = 180;

        // TODO: Use these values
        public static final double PIVOT_DEADBAND_DEGREES = 2;

        public static final double PIVOT_LOW_LIMIT_DEGREES = 90 - 20;
        public static final double PIVOT_HIGH_LIMIT_DEGREES = 90 + 155;

        public static final int PIVOT_LEAD_CAN_ID = 5;
        public static final int PIVOT_FOLLOWER_CAN_ID = 6;

        public static final double PIVOT_P = 0.005;
        public static final double PIVOT_I = 0;
        public static final double PIVOT_D = 0.0725;

        public static final int LOW_INDEX = 0;
        public static final int MID_INDEX = 1;
        public static final int HIGH_INDEX = 2;
        public static final int RESET_INDEX = 3;

        // Multiply all encoder outputs by this factor to get degrees
        public static final int PIVOT_POSITION_ENCODER_FACTOR = 360;
        public static final int PIVOT_SMART_CURRENT_LIMIT = 30;
    }

    public final static class ClawConstants {

        // The outtake limit for the claw (its 10 so there is basically nothing limiting
        // it)
        public static final double SLEW_NEGATIVE_LIMIT = -10;
        // The intake limit for the claw (we want to prevent skipping so we have a very
        // low limit)
        public static final double SLEW_POSITIVE_LIMIT = 0.75;

        public static final int CLAW_LEAD_CAN_ID = 8;
        public static final int CLAW_FOLLOWER_CAN_ID = 7;

        public static final int CLAW_FREE_CURRENT_LIMIT = 15;
        public static final int CLAW_STALL_CURRENT_LIMIT = 5;
        public static final int CLAW_OUTTAKE_CURRENT_LIMIT = 40;

    }

    public static final class AlignmentConstants {
        public static final double CHARGE_PAD_CORRECTION_P = .1;
        public static final double LOW_SPEED = .1;
        public static final double HIGH_SPEED = .3;
        public static final double ZERO_OFFSET = 7;
    }

    public static final class PlacementConstants {
        public static final int LOW_INDEX = 0;
        public static final int MID_INDEX = 1;
        public static final int HIGH_INDEX = 2;
        public static final int RESET_INDEX = 3;

        public static final double LOW_PLACEMENT_FRONT = 90 + 0;
        public static final double MID_PLACEMENT_FRONT = 90 + 40;
        public static final double HIGH_PLACEMENT_FRONT = 90 + 60;
        public static final double RESET_PLACEMENT = 78;

        public static final double[] PLACEMENT_POSITIONS_FRONT = {
                LOW_PLACEMENT_FRONT,
                MID_PLACEMENT_FRONT,
                HIGH_PLACEMENT_FRONT,
                RESET_PLACEMENT
        };

        // TODO: Use these values
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

        public static final double LOW_PLACEMENT_BACK = 270 + 0;
        public static final double MID_PLACEMENT_BACK = 270 - 50;
        public static final double HIGH_PLACEMENT_BACK = 270 - 60;

        public static final double[] PLACEMENT_POSITIONS_BACK = {
                LOW_PLACEMENT_BACK,
                MID_PLACEMENT_BACK,
                HIGH_PLACEMENT_BACK,
                RESET_PLACEMENT
        };

        // TODO: Use these values
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
        
        public static final double INTAKE_SPEED = 0.4;
    }

    public static final class VisionConstants {
        
    }
    
    public static final class LEDConstants {
      public static final int ARDUINO_ADDRESS = 8;

      public static final int MOVING_DOTS = 1;
      public static final int RAINBOW_FULL = 2;
      public static final int GREEN_WHITE_GOLD = 3;
      public static final int RED_FIRE = 4;
      public static final int BLUE_FIRE = 5;
    }
}
