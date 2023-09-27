package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveToPoint extends CommandBase{

    private final Drivetrain drivetrain;

    private final Pose2d startPose;
    private final Translation2d endTranslation;
    private final double dt;

    private Twist2d twist;

    private Rotation2d endRotation;


    public DriveToPoint(Pose2d startPose, Translation2d endTranslation, double dt, Drivetrain drivetrain){
        this.drivetrain = drivetrain;
        this.startPose = startPose;
        this.endTranslation = endTranslation;
        this.dt = dt;

        double dx = endTranslation.getX() - startPose.getTranslation().getX();
        double dy = endTranslation.getY() - startPose.getTranslation().getY();

        endRotation = new Rotation2d( -(startPose.getRotation().getDegrees()) + Math.atan2(dx, dy) );

        addRequirements(drivetrain);
    }
    
    @Override
    public void initialize(){
        twist = startPose.log(new Pose2d(endTranslation.getX(), endTranslation.getY(), endRotation));
        double dxt = twist.dx / dt;
        double dyt = twist.dy / dt;
        double dtheta = twist.dtheta / dt;
        twist = new Twist2d(dxt, dyt, dtheta);

        drivetrain.drive(Math.abs((twist.dx - twist.dy)), twist.dtheta);
    }




    @Override
    public void end(boolean interrupted){
        drivetrain.drive(0, 0);
    }

    @Override
    public boolean isFinished(){
        if (startPose.getTranslation().getDistance(endTranslation) < 0.1){
            return true;
        }
        return false;
    }
    
}
