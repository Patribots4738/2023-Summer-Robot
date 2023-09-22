package frc.team3128.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ProxyScheduleCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.limelight.LimelightBase;

import java.util.function.Supplier;

/**
 * Class to store information about autonomous routines.
 * @author Daniel Wang, Mason Lam
 */

public class AutoPrograms {

    private Drivetrain drive;
    private LimelightBase limelights;

    public AutoPrograms() {
        drive = Drivetrain.getInstance();
        limelights = LimelightBase.getInstance();
        initAutoSelector();
    }

    private void initAutoSelector() {
        String[] autoStrings = new String[] {"Intake 0", "Intake 1", "Intake 2", "Intake 1 Hoard 2"};
        //TODO: add to autoStrings to dashboard
        
    }

    public Command getAutonomousCommand() {
        //TODO: get selected auto from dashboard
        String selectedAutoName = .getSelectedAutoName();

        if (selectedAutoName == null) {
            selectedAutoName = "Intake 0";
        }

        Pose2d initialPose = null;
        Command autoCommand = null;

        switch (selectedAutoName) {
            case "Intake 0":
            
                break;

            default:
                //TODO: see if this will be an issue
                //TODO: what is Log
                Log.info("Auto Selector", "Something went wrong in getting the auto name - misspelling?");
                break;
        }

        drive.getOdometry().resetPose(initialPose);
        return autoCommand;
    }

    /**
     * Flip 180 degrees rotation wise but keep same pose translation 
     */
    private Pose2d inverseRotation(Pose2d pose) {
        return new Pose2d(pose.getTranslation(), new Rotation2d(pose.getRotation().getRadians() + Math.PI));
    }
}