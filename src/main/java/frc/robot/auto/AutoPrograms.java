package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.limelight.LimelightBase;

import java.util.HashMap;
import java.util.Set;

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
    }

    private void initAutoSelector() {
        auto = new HashMap<String, Command>();

        Trajectories.autoSpeed = 2.5;
        //TODO: add to autoStrings to dashboard

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
            return auto.get("DEFAULT");
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