package frc.robot.commands.auto;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
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

  public LoadAutoBuilder(Drivetrain drivetrain, HashMap<String, Command> eventMap,
      ArrayList<PathPlannerTrajectory> trajectory) {

    this.eventMap = eventMap;
    this.drivetrain = drivetrain;
    this.kinematics = this.drivetrain.getKinematics();
    this.odometry = this.drivetrain.getOdometry();
    this.trajectory = trajectory;

    this.autoBuilder = new RamseteAutoBuilder(
        this.odometry::getPoseMeters,
        this.odometry::resetPosition,
        this.drivetrain.getRamseteController(),
        this.kinematics,
        this.drivetrain::setOutputVolts,
        this.eventMap,
        true,
        this.drivetrain);
  }

  public CommandBase getAuto() {
    return autoBuilder.fullAuto(trajectory);
  }
}
