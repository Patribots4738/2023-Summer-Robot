package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.PlacementConstants;
import frc.robot.commands.AutoLevel;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.limelight.LimelightBase;

import java.util.HashMap;
import java.util.Set;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

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
        CommandBase placeBackHigh = 
            Commands.run(() -> Pivot.getInstance().setArmHigh(true))
            .until(Pivot.getInstance()::pivotAtDesiredPosition)
            .andThen(() -> Claw.getInstance().setSpeed(PlacementConstants.HIGH_INDEX, true))
            .andThen(new WaitCommand(0.7))
            .andThen(() -> Pivot.getInstance().setArmReset())
            .andThen(() -> Claw.getInstance().setSpeed(0));
              
        placeBackHigh.addRequirements(Pivot.getInstance());
        placeBackHigh.addRequirements(Claw.getInstance());
        
        Command mobilityChargeTrajectory = loadPath("5_M_CH", constraints);
        
        AutoLevel autoLevel = new AutoLevel();
        
        SequentialCommandGroup P1_Mobility_Charge = new SequentialCommandGroup(
            placeBackHigh.asProxy(),
            mobilityChargeTrajectory.asProxy(),
            autoLevel.asProxy()
        );

        CommandBase intakeClaw = Commands.runOnce(() -> Claw.getInstance().setSpeed(PlacementConstants.INTAKE_SPEED));

        CommandBase twoPTrajectory = loadPath("2P", constraints).alongWith(intakeClaw);
        CommandBase mobilityTrajectory = loadPath("M", constraints);

        CommandBase placeBackMid = 
            Commands.run(() -> Pivot.getInstance().setArmMid(true))
            .until(Pivot.getInstance()::pivotAtDesiredPosition)
            .andThen(() -> Claw.getInstance().setSpeed(PlacementConstants.RESET_INDEX, false))
            .andThen(() -> Claw.getInstance().setSpeed(PlacementConstants.MID_INDEX, true))
            .andThen(new WaitCommand(0.7))
            .andThen(() -> Pivot.getInstance().setArmReset())
            .andThen(() -> Claw.getInstance().setSpeed(0));

        SequentialCommandGroup twoP = new SequentialCommandGroup(
            placeBackHigh.asProxy(),
            twoPTrajectory.asProxy(),
            placeBackMid.asProxy(),
            mobilityTrajectory.asProxy()
        );



        auto.put("DEFAULT", (placeBackHigh.asProxy()
        .andThen(Commands.run(() -> Drivetrain.getInstance().drive(-0.6, 0)).asProxy()
        .withTimeout(4)
        .alongWith(Commands.runOnce(() -> Claw.getInstance().setSpeed(0.3)))
        .andThen(Commands.runOnce(() -> Drivetrain.getInstance().drive(0, 0))
        .andThen(Commands.runOnce(() -> Claw.getInstance().setSpeed(0)))
        ))));
        


        // auto.put("2P", twoP);
        // auto.put("mobilityCharge", P1_Mobility_Charge);

        Set<String> keySet = auto.keySet();
        String[] keySetCopy = new String[keySet.size()];

        int index = 0;
        for(String x : keySet){
            keySetCopy[index] = x;
            index++;
        }
        
        for (int i = 1; i < auto.size(); i++){
            autoList.addOption(keySetCopy[i], keySetCopy[i]);
        }
    }

    public Command getAutonomousCommand() {
        Commands.runOnce(() -> Drivetrain.getInstance().drive(0, 0), Drivetrain.getInstance());
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

    public SequentialCommandGroup loadPath(String pathName, PathConstraints constraints) {

        PathPlannerTrajectory trajectory = PathPlanner.loadPath(pathName, constraints);
        PPRamseteCommand command = new PPRamseteCommand(
          trajectory,
          Drivetrain.getInstance()::getPose,
          Drivetrain.getInstance().getRamseteController(),
          // Drivetrain.getInstance().getFeedforward(),
          Drivetrain.getInstance().getKinematics(),
          // Drivetrain.getInstance()::getWheelSpeeds,
          // new PIDController(1, 0, 0),
          // new PIDController(1, 0, 0),
          Drivetrain.getInstance()::tankDriveVolts,
          Drivetrain.getInstance()
        );

        // Reset the pose to the initial pose of the trajectory
        // Run the trajectory
        // Stop the drive
        return new SequentialCommandGroup(
            Commands.runOnce(() -> Drivetrain.getInstance()
            .getOdometry()
            .resetPose(
                Drivetrain.getInstance()
                    .getYaw(), 
                    trajectory.getInitialPose())
            ),
          command
          );
    }
}