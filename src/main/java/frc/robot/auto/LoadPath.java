package frc.robot.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class LoadPath {
    
    private PathConstraints constraints;
    private PathPlannerTrajectory trajectory;
    private RamseteCommand command;

    public LoadPath() {
        this.constraints = new PathConstraints(DrivetrainConstants.MAX_DRIVE_VELOCITY, DrivetrainConstants.MAX_DRIVE_ACCELERATION);
        this.command = loadPath("DEFAULT", this.constraints);
    }

    public RamseteCommand loadPath(String pathName, PathConstraints constraints) {

        this.trajectory = PathPlanner.loadPath(pathName, constraints);
        this.command = new RamseteCommand(
            this.trajectory, 
            Drivetrain.getInstance()::getPose, 
            Drivetrain.getInstance().getRamseteController(),
            Drivetrain.getInstance().getFeedforward(),
            Drivetrain.getInstance().getKinematics(),
            Drivetrain.getInstance()::getWheelSpeeds,
            Drivetrain.getInstance().getLeftPIDController(),
            Drivetrain.getInstance().getRightPIDController(),
            Drivetrain.getInstance()::tankDriveVolts,
            Drivetrain.getInstance());

        return this.command;        
    }

}
