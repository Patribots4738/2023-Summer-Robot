package frc.robot.auto;

import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

/**
 * Store trajectories for autonomous. Edit points here. 
 * @author Daniel Wang
 */
public class Trajectories {
    private static HashMap<String, List<PathPlannerTrajectory>> trajectories = new HashMap<String, List<PathPlannerTrajectory>>();

    private static RamseteAutoBuilder builder;

    private static HashMap<String, Command> CommandEventMap = new HashMap<String, Command>();

    private static Drivetrain drivetrain = Drivetrain.getInstance();

    public static double autoSpeed = DrivetrainConstants.MAX_DRIVE_SPEED;

    public static void initTrajectories() {
        final String[] trajectoryNames = {

        };
        
        // TODO: put in Commands for automous
        

        for (String trajectoryName : trajectoryNames) {
            trajectories.put(trajectoryName, PathPlanner.loadPathGroup(trajectoryName, new PathConstraints(DrivetrainConstants.MAX_DRIVE_SPEED, DrivetrainConstants.MAX_DRIVE_ACCELERATION)));
        }

        builder = new RamseteAutoBuilder(
            Drivetrain.getInstance()::getPose,
            Drivetrain.getInstance().getOdometry()::resetPose,
            Drivetrain.getInstance().getRamseteController(),
            DrivetrainConstants.DRIVE_KINEMATICS,
            Drivetrain.getInstance()::setOutputSpeeds,
            CommandEventMap,
            Drivetrain.getInstance()
        );
    }

    public static CommandBase get(String name){
        return builder.fullAuto(trajectories.get(name));
    }

    public static PathPlannerTrajectory line(Pose2d start, Pose2d end){
        return PathPlanner.generatePath(
            new PathConstraints(DrivetrainConstants.MAX_DRIVE_SPEED, DrivetrainConstants.MAX_DRIVE_ACCELERATION),
            new PathPoint(start.getTranslation(), start.getRotation()),
            new PathPoint(end.getTranslation(), end.getRotation())
        );
    }

    public static CommandBase lineCmd(Pose2d start, Pose2d end){
        return builder.fullAuto(line(start, end));
    }
}