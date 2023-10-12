package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.AutoLevel;
import frc.robot.commands.ResetPose;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.limelight.LimelightBase;
import java.util.HashMap;
import java.util.Set;

import com.pathplanner.lib.PathConstraints;

/**
 * Class to store information about autonomous routines.
 * @author Daniel Wang, Mason Lam
 */

public class AutoPrograms {

    private HashMap<String, Command> auto;
    public Drivetrain drive;
    public LimelightBase limelights;

    private static SendableChooser<String> autoList = new SendableChooser<>();

    public AutoPrograms() {
        drive = Drivetrain.getInstance();
        limelights = LimelightBase.getInstance();

        
        initAutoSelector();
        SmartDashboard.putData("Auto choices", autoList);
    }

    private void initAutoSelector() {
        auto = new HashMap<String, Command>();
    
        PathConstraints constraints = new PathConstraints(DrivetrainConstants.MAX_DRIVE_VELOCITY, DrivetrainConstants.MAX_DRIVE_ACCELERATION);

        // TODO: add more auto programs here
        constraints = new PathConstraints(2, DrivetrainConstants.MAX_DRIVE_ACCELERATION);
        CommandBase placeBackHigh = Commands.run(() -> Pivot.getInstance().setArmHigh(true))
            .until(Pivot.getInstance()::pivotAtDesiredPosition);
        placeBackHigh.addRequirements(Pivot.getInstance());
        
        Command trajCommand = LoadPath.loadPath("P1_Mobility_Charge", constraints);

              AutoLevel autoLevel = new AutoLevel();
        SequentialCommandGroup P1_Mobility_Charge = new SequentialCommandGroup(
            placeBackHigh,
            // trajCommand,
            // Commands.run(() -> Drivetrain.getInstance().drive(1, 0)).withTimeout(3)
            autoLevel
        );

        auto.put("DEFAULT", P1_Mobility_Charge);

        Set<String> keySet = auto.keySet();
        String[] keySetCopy = new String[keySet.size()];

        int index = 0;
        for(String x : keySet){
            keySetCopy[index] = x;
            index++;
        }
        
        for (int i = 0; i < auto.size(); i++){
            autoList.addOption(keySetCopy[i], keySetCopy[i]);
        }
    }

    public Command getAutonomousCommand() {
        String selectedAutoName = autoList.getSelected();

        if(selectedAutoName == null){
            selectedAutoName = "DEFAULT";
        }

        return auto.get(selectedAutoName);
    }

    /**
     * Flip 180 degrees rotation wise but keep same pose translation 
     */
    private Pose2d inverseRotation(Pose2d pose) {
        return new Pose2d(pose.getTranslation(), new Rotation2d(pose.getRotation().getRadians() + Math.PI));
    }
}