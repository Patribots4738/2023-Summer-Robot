package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.DriveToPoint;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.limelight.LimelightBase;
import io.github.oblarg.oblog.annotations.Log;

import java.util.HashMap;
import java.util.Set;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

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

        Trajectories.autoSpeed = 2.5;
        //TODO: add to autoStrings to dashboard
        auto.put("DEFAULT", new DriveToPoint(Drivetrain.getInstance().getPose(), 
                new Translation2d(1.0, 1.0), 
                4.0, 
                Drivetrain.getInstance()));

        Trajectories.autoSpeed = 2.5;
        PathConstraints constraints = new PathConstraints(DrivetrainConstants.MAX_DRIVE_VELOCITY, DrivetrainConstants.MAX_DRIVE_ACCELERATION);

        RamseteCommand command = new RamseteCommand(
          PathPlanner.loadPath("Charge", constraints), 
          Drivetrain.getInstance()::getPose, 
          Drivetrain.getInstance().getRamseteController(),
          Drivetrain.getInstance().getFeedforward(),
          Drivetrain.getInstance().getKinematics(),
          Drivetrain.getInstance()::getWheelSpeeds,
          Drivetrain.getInstance().getLeftPIDController(),
          Drivetrain.getInstance().getRightPIDController(),
          Drivetrain.getInstance()::tankDriveVolts,
          Drivetrain.getInstance());

        auto.put("BASIC_CHARGE", command.andThen(() -> Drivetrain.getInstance().drive(0, 0)));

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
      Drivetrain.getInstance().getOdometry().resetPose(new Pose2d());
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