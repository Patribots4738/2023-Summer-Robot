package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

public class ResetPose extends CommandBase {
    private static Pose2d desiredPose = new Pose2d();
    private static InstantCommand command = 
        new InstantCommand(() -> Drivetrain.getInstance()
            .getOdometry()
            .resetPose(
                Drivetrain.getInstance()
                    .getYaw(), 
                    desiredPose)
    );
    private static ResetPose instance;

    public ResetPose(){
      addRequirements(Drivetrain.getInstance());
    }

    public static ResetPose setDesiredPose(Pose2d pose){
        desiredPose = pose;
        return ResetPose.getInstance();
    }

    private static ResetPose getInstance() {
      if(instance == null){
        instance = new ResetPose();
      }

      return instance;
    }

    public InstantCommand getCommand(){
      return command;
    }

    @Override
    public void initialize() {
        command.execute();
    }
}
