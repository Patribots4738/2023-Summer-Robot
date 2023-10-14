package frc.robot.commands;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Odometry extends CommandBase {
  private final Drivetrain drive;
  private final DifferentialDriveOdometry odometry;
  private Rotation2d rotation;
  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;

  public Odometry(Drivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.rotation = drive.getRotation2d();
    this.leftEncoder = drive.getLeftEncoder();
    this.rightEncoder = drive.getRightEncoder();
    this.odometry = new DifferentialDriveOdometry(rotation,
        leftEncoder.getPosition(), rightEncoder.getPosition());

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    odometry.resetPosition(new Rotation2d(), 0, 0, new Pose2d());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    odometry.update(rotation, leftEncoder.getPosition(), rightEncoder.getPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    odometry.resetPosition(new Rotation2d(), 0, 0, new Pose2d());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * Returns the current pose in meters.
   * 
   * @return The method is returning a Pose2d object.
   */
  public Pose2d getPoseMeters() {
    return odometry.getPoseMeters();
  }

  /**
   * Returns the rotation of the pose in meters.
   * 
   * @return The method is returning a Rotation2d object.
   */
  public Rotation2d getRotation() {
    return odometry.getPoseMeters().getRotation();
  }

  /**
   * Returns the DifferentialDriveOdometry object.
   * 
   * @return The method is returning an instance of the DifferentialDriveOdometry class.
   */
  public DifferentialDriveOdometry getOdometry() {
    return odometry;
  }

}
