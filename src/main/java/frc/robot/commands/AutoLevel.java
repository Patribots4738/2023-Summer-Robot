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
    
    Supplier<DoubleSupplier> getTilt;

    public AutoLevel(){
        this.elapsedTime = new Timer();

        this.getTilt = () -> {
            if (-90 < Drivetrain.getInstance().getYaw().getDegrees() && Drivetrain.getInstance().getYaw().getDegrees() < 90) {
              tiltedAngle = () -> -Drivetrain.getInstance().getRoll().getDegrees();
            }
            else if (90 < Drivetrain.getInstance().getYaw().getDegrees() && Drivetrain.getInstance().getYaw().getDegrees() < 180) 
            {
              tiltedAngle = () -> Drivetrain.getInstance().getRoll().getDegrees();
            }

            // System.out.println(
            //   "Pitch: "+ Drivetrain.getInstance().getPitch().getDegrees() 
            //   + ", Roll : " + tiltedAngle);
            
            return tiltedAngle;
        };

        this.driveForward = new InstantCommand(
            () -> Drivetrain.getInstance().drive(
                MathUtil.clamp(
                    ((AlignmentConstants.CHARGE_PAD_CORRECTION_P * getTilt.get().getAsDouble())
                     / (1/*elapsedTime.get()*/ /(DriverStation.isAutonomous() 
                        ? 10 : 20))), 
                    0.055, 
                    0.20), 
                    0));

        this.driveBackward = new InstantCommand(
            () -> Drivetrain.getInstance().drive(
                MathUtil.clamp(
                    ((AlignmentConstants.CHARGE_PAD_CORRECTION_P * getTilt.get().getAsDouble())
                     / (1/*elapsedTime.get()*/ / (DriverStation.isAutonomous() 
                        ? 10 : 20))), 
                        -0.20, 
                        -0.055),
                        0));

        ConditionalCommand command2 =  new ConditionalCommand(
          this.driveBackward.until(tiltedBackwards), 
          new InstantCommand(() -> Drivetrain.getInstance().drive(0, 0)),
          // StopDrive.getCommand(), 
          tiltedForward);

        conditional = new ConditionalCommand(
            this.driveForward.until(tiltedForward),
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

        this.tiltedForward = () -> tiltedAngle.getAsDouble() > 7;
        this.tiltedBackwards = () -> tiltedAngle.getAsDouble() < -7;

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
