package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class DriveWithRoation extends SequentialCommandGroup {

  public DriveWithRoation(Drivetrain drivetrain, double distance, double angle) {
    addCommands(
        new RotateBot(drivetrain, angle)
        .andThen(new DriveToDistance(drivetrain, distance)));
  }
}
