package frc.robot.auto;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.DriveToPoint;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class Trajectories {

    private static HashMap<String, List<PathPlannerTrajectory>> trajectories = new HashMap<String, List<PathPlannerTrajectory>>();

    private static RamseteAutoBuilder builder;

    private static HashMap<String, Command> CommandEventMap = new HashMap<String, Command>();

    private static Drivetrain drivetrain = Drivetrain.getInstance();

    // TODO: add max speed
    public static double autoSpeed = DrivetrainConstants.MAX_SPEED;

    // private static Intake intake = Intake.getInstance();

    public static void initTrajectories() {
        // TODO: add trajectories
        final String[] trajectoryNames = { "" };

        for (String name : trajectoryNames) {
            trajectories.put(name, PathPlanner.loadPathGroup(
                                        name, 
                                        new PathConstraints(DrivetrainConstants.MAX_SPEED, 
                                                            DrivetrainConstants.AUTO_SPEED)));
        }

        builder = LoadAutoBuilder.getInstance().getAutoBuilder();    
    }

    public static CommandBase get(String name) {
        return builder.fullAuto(trajectories.get(name));
    }

    public static PathPlannerTrajectory line(Pose2d start, Pose2d end) {
        return PathPlanner.generatePath(
                new PathConstraints(DrivetrainConstants.MAX_SPEED, DrivetrainConstants.MAX_ACCELERATION),
                new PathPoint(start.getTranslation(), start.getRotation()),
                new PathPoint(end.getTranslation(), end.getRotation()));
    }

    public static CommandBase lineCmd(Pose2d start, Pose2d end) {
        return builder.fullAuto(line(start, end));
    }

    public static CommandBase resetOdometry() {
        return Commands.sequence(
            new InstantCommand(()-> Vision.AUTO_ENABLED = true),
            new InstantCommand(()-> drivetrain.zeroGyro((DriverStation.getAlliance() == Alliance.Red) || (DriverStation.getAlliance() == Alliance.Blue) ? 0 : 180)),
            new DriveToPoint(new Translation2d(DriverStation.getAlliance() == Alliance.Red ? -2.5 : 2.5, 0), new Rotation2d(), drivetrain)
                .until(() -> Vision.getInstance().getCamera(VisionConstants.FRONT).hasValidTarget()),
            new InstantCommand(()-> drivetrain.stop(), drivetrain),
            new InstantCommand(()-> Vision.getInstance().visionReset())
        ).withTimeout(2);
    }

}
