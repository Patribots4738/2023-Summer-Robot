package frc.robot.commands;

import java.util.Set;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.subsystems.Drivetrain;

public class DriveToPoint extends RunCommand{

    private Drivetrain drivetrain;
    private Translation2d translation;
    private Rotation2d rotation;

    public DriveToPoint(Translation2d translation, Rotation2d rotation, Drivetrain drivetrain){
        super(() -> drivetrain.drive(), drivetrain);

        this.translation = translation;
        this.rotation = rotation;
        this.drivetrain = drivetrain;
        
        addRequirements(drivetrain);
    }
    
}
