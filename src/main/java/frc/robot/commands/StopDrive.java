package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

public class StopDrive extends CommandBase {
    private static Command command = new InstantCommand(() -> Drivetrain.getInstance().drive(0, 0));
    
    public StopDrive(){
        addRequirements(Drivetrain.getInstance());
    }
    
    @Override
    public void initialize() {
        command.execute();
    }

    public static Command getCommand(){
      return command;
    }
    
}
