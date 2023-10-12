package frc.robot.commands;

import java.util.function.BiConsumer;
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

    boolean tiltedForward;
    boolean tiltedBackwards;

    private double tilt;
    
    Supplier<Double> getTilt;

    public AutoLevel(){
        this.elapsedTime = new Timer();

        this.getTilt = () -> {
            double tiltedAngle = 0;
            if (-90 < Drivetrain.getInstance().getYaw().getDegrees() && Drivetrain.getInstance().getYaw().getDegrees() < 90) {
                tiltedAngle =  Drivetrain.getInstance().getPitch().getRadians();
            }
            else
            {
                tiltedAngle = -Drivetrain.getInstance().getPitch().getRadians();
            }
            return tiltedAngle;
        };
        
        addRequirements(Drivetrain.getInstance());
    }

    // TODO: Do we need to use pitch and roll as shown above?

    @Override
    public void initialize() {
        this.elapsedTime.start();
    }

    @Override
    public void execute() {
        this.tilt = this.getTilt.get();

        this.tiltedForward = tilt < 7;
        this.tiltedBackwards = tilt > -7;

        this.driveForward = new InstantCommand(
            () -> Drivetrain.getInstance().drive(
                MathUtil.clamp(
                    ((AlignmentConstants.CHARGE_PAD_CORRECTION_P * tilt)
                     / (elapsedTime.get() /(DriverStation.isAutonomous() 
                        ? 10 : 20))), 
                    0.055, 
                    0.20), 
                    0));

        this.driveBackward = new InstantCommand(
            () -> Drivetrain.getInstance().drive(
                MathUtil.clamp(
                    ((AlignmentConstants.CHARGE_PAD_CORRECTION_P * tilt)
                     / (elapsedTime.get() / (DriverStation.isAutonomous() 
                        ? 10 : 20))), 
                        -0.20, 
                        -0.055),
                        0));

        ConditionalCommand command2 =  new ConditionalCommand(
          this.driveBackward.until(() -> tiltedBackwards), 
          new InstantCommand(() -> Drivetrain.getInstance().drive(0, 0)),
          // StopDrive.getCommand(), 
          () -> tiltedForward);

        ConditionalCommand command = new ConditionalCommand(
            this.driveForward.until(() -> tiltedForward),
            command2,
            () -> tiltedBackwards);

        command.initialize();
        command.execute();
    }

    @Override
    public boolean isFinished() {
        return (!tiltedForward && !tiltedBackwards);
    }

}
