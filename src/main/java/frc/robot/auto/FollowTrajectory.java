package frc.robot.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Odometry;

public class FollowTrajectory {

    private PathPlannerTrajectory trajectory;
    private boolean isFirstPath;
    private Drivetrain drivetrain;
    private Odometry odometry;

    /*
     * Follows a trajectory using the drivetrain's ramsete controller
     * 
     * @param drivetrain The drivetrain subsystem to use
     * @param trajectory The trajectory to follow
     * @param isFirstPath Whether or not this is the first path in a sequence (resets odometry if true)
     */
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
