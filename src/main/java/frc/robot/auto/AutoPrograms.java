package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.StopDrive;
import frc.robot.subsystems.Drivetrain;
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
        //TODO: add to autoStrings to dashboard

        constraints = new PathConstraints(2, DrivetrainConstants.MAX_DRIVE_ACCELERATION);
        SequentialCommandGroup command = LoadPath.loadPath("Charge", constraints);
        auto.put("DEFAULT", new SequentialCommandGroup(command, StopDrive.getCommand()));

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