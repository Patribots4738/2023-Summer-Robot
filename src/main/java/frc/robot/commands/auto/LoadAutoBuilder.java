package frc.robot.commands.auto;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Odometry;
import frc.robot.subsystems.Drivetrain;

public class LoadAutoBuilder {

  private DifferentialDriveKinematics kinematics;
  private HashMap<String, Command> eventMap;
  private RamseteAutoBuilder autoBuilder;
  private Drivetrain drivetrain;
  private Odometry odometry;
  private ArrayList<PathPlannerTrajectory> trajectory;

  public LoadAutoBuilder(Drivetrain drivetrain, DifferentialDriveKinematics kinematics,
      Odometry odometry, XboxController controller, 
      HashMap<String, Command> eventMap, ArrayList<PathPlannerTrajectory> trajectory) {
    
    this.kinematics = kinematics;
    this.eventMap = eventMap;
    this.odometry = odometry;
    this.drivetrain = drivetrain;
    this.trajectory = trajectory;

    this.autoBuilder = new RamseteAutoBuilder(
        this.odometry::getPoseMeters,                // The odometry object that contains the current position of the robot
        this.odometry::resetPosition,                // A method that resets the odometry object to a given position
        new RamseteController(2, 0.7),        // The Ramsete Controller
        this.kinematics,                             // The kinematics object that contains the track width and wheel radius
        (Double leftOutput, Double rightOutput) -> { // A method that sets the speeds of the left and right motors
          this.drivetrain.setSpeeds(
          new DifferentialDriveWheelSpeeds(leftOutput, rightOutput));
        },
        this.eventMap,                               // The event map that contains the events that will be triggered during the auto
        true,                       // Whether or not the auto path should be swapped based on color
        this.drivetrain);                            // The drivetrain object
  }

  public CommandBase getAuto(){
    return this.autoBuilder.fullAuto(trajectory);
  }
}
