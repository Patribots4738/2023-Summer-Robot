package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.WPIMathJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class DriveToPoint extends CommandBase{

    private Drivetrain drivetrain;

    private Pose2d startPose;
    private Pose2d endPose;

    public Twist2d twist;

    public DriveToPoint(Pose2d startPose, Pose2d endPose, Drivetrain drivetrain){
        this.drivetrain = drivetrain;

        this.startPose = startPose;
        this.endPose = endPose;
        
        addRequirements(drivetrain);
    }
    
    @Override
    public void initialize(){
        drivetrain.getOdometry().resetPose(startPose);

        // get the velocities of the robot to get to the end pose
        twist = startPose.log(endPose);

        Translation2d translation = new Translation2d(twist.dx, twist.dy);

        double norm = 1;
        if (startPose.getTranslation().getNorm() < endPose.getTranslation().getNorm())
          norm = -1;
        drivetrain.drive(( translation.getNorm() * norm )/0.02, twist.dtheta/0.02);
          
    }


    @Override
    public void end(boolean interrupted){
        drivetrain.drive(0, 0);
    }

    @Override
    public boolean isFinished(){
        boolean isClose = Math.abs(drivetrain.getPose().minus(endPose).getTranslation().getNorm()) < 0.1;

        return isClose;
    }


    
}
