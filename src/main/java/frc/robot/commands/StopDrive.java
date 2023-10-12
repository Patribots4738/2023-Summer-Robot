package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

public class StopDrive extends InstantCommand {
    private static Runnable command = () -> Drivetrain.getInstance().drive(0, 0);
    
    public StopDrive(){
        super(command);
    }

    public InstantCommand getCommand(){
      return this;
    }
    
}
