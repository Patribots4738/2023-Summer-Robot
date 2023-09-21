package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class RotateBot extends CommandBase {

    private final Drivetrain drivetrain;
    private double angle;
    private boolean isFinished = false;

    public RotateBot(Drivetrain drivetrain, double angle) {
        this.drivetrain = drivetrain;
        this.angle = angle;

        addRequirements(this.drivetrain);
    }

    @Override
    public void initialize() {
        // rotate to angle
        drivetrain.drive(0, angle);
    }

    @Override
    public void execute() {
        // check if at angle
        if (drivetrain.getAngle() - angle < DrivetrainConstants.ANGLE_TOLERANCE) {
            isFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        // Return true when the command should end.
        return this.isFinished;
    }
}
