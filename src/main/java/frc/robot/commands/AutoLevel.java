package frc.robot.commands;

import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.Constants.AlignmentConstants;
import frc.robot.subsystems.Drivetrain;

public class AutoLevel extends CommandBase{

    Timer elapsedTime;

    Command driveForward;
    Command driveBackward;
    Command conditional;

    BooleanSupplier tiltedForward;
    BooleanSupplier tiltedBackwards;

    private DoubleSupplier tiltedAngle;
    private DoubleSupplier yaw;
    
    Supplier<DoubleSupplier> getTilt;

    public AutoLevel(){
        this.elapsedTime = new Timer();

        this.getTilt = () -> {
            yaw = () -> Drivetrain.getInstance().getYaw().getDegrees();
            if (-90 < yaw.getAsDouble() && yaw.getAsDouble() < 90) {
              tiltedAngle = () -> -Drivetrain.getInstance().getRoll().getDegrees();
            }
            else if (90 < yaw.getAsDouble() && yaw.getAsDouble() < 180) 
            {
              tiltedAngle = () -> Drivetrain.getInstance().getRoll().getDegrees();
            }
            
            return tiltedAngle;
        };
        this.tiltedAngle = this.getTilt.get();

        
        this.tiltedBackwards = () -> tiltedAngle.getAsDouble() < -AlignmentConstants.ZERO_OFFSET;
        this.tiltedForward = () -> tiltedAngle.getAsDouble() > AlignmentConstants.ZERO_OFFSET;

        this.driveForward = new InstantCommand(
            () -> Drivetrain.getInstance().drive(
                MathUtil.clamp(
                    (AlignmentConstants.CHARGE_PAD_CORRECTION_P * 11), 
                    AlignmentConstants.LOW_SPEED, 
                    AlignmentConstants.HIGH_SPEED), 
                    0));

        this.driveBackward = new InstantCommand(
            () -> Drivetrain.getInstance().drive(
                MathUtil.clamp(
                    (AlignmentConstants.CHARGE_PAD_CORRECTION_P * -11), 
                        -AlignmentConstants.HIGH_SPEED, 
                        -AlignmentConstants.LOW_SPEED),
                        0));

        ConditionalCommand command2 =  new ConditionalCommand(
          this.driveForward, 
          new InstantCommand(() -> Drivetrain.getInstance().drive(0, 0)),
          // StopDrive.getCommand(), 
          tiltedBackwards);

        conditional = new ConditionalCommand(
            this.driveBackward,
            command2,
            tiltedForward);
        
        addRequirements(Drivetrain.getInstance());
    }

    @Override
    public void initialize() {
        this.elapsedTime.restart();
        conditional.initialize();
    }

    @Override
    public void execute() {
        this.tiltedAngle = this.getTilt.get();
        conditional.initialize();
        conditional.execute();
    }

    @Override
    public boolean isFinished() {
        return false;
        // return (!tiltedForward && !tiltedBackwards);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        elapsedTime.stop();
        elapsedTime.reset();
    }
}
