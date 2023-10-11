package frc.robot.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.PlacementConstants;
import frc.robot.commands.ResetPose;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pivot;

public class LoadPath {
    
    private static PathConstraints constraints;
    private static PathPlannerTrajectory trajectory;
    private static PPRamseteCommand command;

    public LoadPath() {
      init();
    }

    private static void init(){
      LoadPath.constraints = new PathConstraints(DrivetrainConstants.MAX_DRIVE_VELOCITY, DrivetrainConstants.MAX_DRIVE_ACCELERATION);
  }

    public static SequentialCommandGroup loadPath(String pathName, PathConstraints constraints) {

        LoadPath.trajectory = PathPlanner.loadPath(pathName, constraints);
        LoadPath.command = new PPRamseteCommand(
          LoadPath.trajectory,
          Drivetrain.getInstance()::getPose,
          Drivetrain.getInstance().getRamseteController(),
          // Drivetrain.getInstance().getFeedforward(),
          Drivetrain.getInstance().getKinematics(),
          // Drivetrain.getInstance()::getWheelSpeeds,
          // new PIDController(1, 0, 0),
          // new PIDController(1, 0, 0),
          Drivetrain.getInstance()::tankDriveVolts,
          Drivetrain.getInstance()
        );

        return new SequentialCommandGroup(
          ResetPose.setDesiredPose(LoadPath.trajectory.getInitialPose()).getCommand(), 
          
          new SequentialCommandGroup(
            new InstantCommand(() -> Pivot.getInstance().setArmHigh(true)))
              .until(Pivot.getInstance()::pivotAtDesiredPosition),
          
          LoadPath.command);
    }

}
