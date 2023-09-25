package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class RotateThenDrive extends SequentialCommandGroup {

    public RotateThenDrive(double speed, double angle, Drivetrain drivetrain ) {
        addCommands(
                new RotateBot(drivetrain, angle)
                        .andThen(() -> drivetrain.tankDriveVolts(0, 0))
                            .andThen(() -> drivetrain.drive(speed,0)));

        addRequirements(drivetrain);
    }
}
