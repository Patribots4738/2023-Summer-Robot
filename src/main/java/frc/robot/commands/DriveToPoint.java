package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveToPoint extends CommandBase{

    private final Drivetrain drivetrain;

    private final Pose2d startPose;
    private final Translation2d endTranslation;
    private final double dt;

    private Twist2d twist;

    private Rotation2d endRotation;

    private Timer timer;


    public DriveToPoint(Pose2d startPose, Translation2d endTranslation, double dt, Drivetrain drivetrain){
        this.drivetrain = drivetrain;
        this.startPose = startPose;
        this.endTranslation = new Translation2d(
            startPose.getTranslation().getX() + endTranslation.getX(), 
            startPose.getTranslation().getY() + endTranslation.getY());
        this.dt = dt;

        double dx = endTranslation.getX() - startPose.getTranslation().getX();
        double dy = endTranslation.getY() - startPose.getTranslation().getY();

        this.endRotation = new Rotation2d( -(startPose.getRotation().getRadians()) + ((2.0) * Math.atan2(dy, dx)) - Math.PI );

        this.timer = new Timer();
        
        addRequirements(drivetrain);
    }
    
    @Override
    public void initialize(){
        twist = startPose.log(new Pose2d(endTranslation.getX(), endTranslation.getY(), endRotation));
        
        double dxt = twist.dx / dt;
        double dyt = twist.dy / dt;
        double dtheta = twist.dtheta / dt;

        twist = new Twist2d(dxt, dyt, dtheta);

        timer.start();

        drivetrain.drive(twist.dx, twist.dtheta);
    }

    @Override
    public void end(boolean interrupted){
        drivetrain.drive(0, 0);
    }

    @Override
    public boolean isFinished(){
        if (timer.hasElapsed(this.dt)){
            timer.stop();
            timer.reset();
            return true;
        }
        return false;
    }
    
}
