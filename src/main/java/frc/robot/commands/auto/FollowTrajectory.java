package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Odometry;
import frc.robot.subsystems.Drivetrain;

public class FollowTrajectory extends CommandBase {

    private PathPlannerTrajectory trajectory;
    private boolean isFirstPath;
    private Drivetrain drivetrain;
    private Odometry odometry;

    public FollowTrajectory(Drivetrain drivetrain, PathPlannerTrajectory trajectory, boolean isFirstPath) {
        this.isFirstPath = isFirstPath;
        this.trajectory = trajectory;
        this.drivetrain = drivetrain;
        this.odometry = drivetrain.getOdometry();
    }

    public Command run() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    if (isFirstPath) {
                        drivetrain.getOdometry().resetPosition(trajectory.getInitialPose());
                    }
                }),
                new PPRamseteCommand(
                        trajectory,
                        odometry::getPoseMeters,
                        drivetrain.getRamseteController(),
                        drivetrain.getKinematics(),
                        drivetrain::setOutputVolts,
                        true,
                        drivetrain));
    }

}
