package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.LoadPath;
import frc.robot.subsystems.Pivot;

public class PlaceHighBackThenTraj extends SequentialCommandGroup{

    Command placeBackHigh = Commands.runOnce(() -> Pivot.getInstance().setArmHigh(true))
                                .until(Pivot.getInstance()::pivotAtDesiredPosition);
    
    SequentialCommandGroup trajCommand;

    public PlaceHighBackThenTraj(String trajName, PathConstraints constraints) {

        this.trajCommand = LoadPath.loadPath(trajName, constraints);

        addCommands(this.placeBackHigh, this.trajCommand);
    }
}
