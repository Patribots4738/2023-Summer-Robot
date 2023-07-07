package frc.robot.commands;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Odometry extends CommandBase {
  private final Drivetrain m_drive;
  private final DifferentialDriveOdometry m_odometry;
  private Rotation2d m_rotation;
  private RelativeEncoder m_leftEncoder;
  private RelativeEncoder m_rightEncoder;

  public Odometry(Drivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_rotation = m_drive.getRotation2d();
    m_leftEncoder = m_drive.getLeftEncoder();
    m_rightEncoder = m_drive.getRightEncoder();
    m_odometry = new DifferentialDriveOdometry(m_rotation,
        m_leftEncoder.getPosition(), m_rightEncoder.getPosition());

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_odometry.resetPosition(new Rotation2d(), 0, 0, new Pose2d());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_odometry.update(m_rotation, m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_odometry.resetPosition(new Rotation2d(), 0, 0, new Pose2d());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public Pose2d getPoseMeters() {
    return m_odometry.getPoseMeters();
  }

  public Rotation2d getRotation() {
    return m_odometry.getPoseMeters().getRotation();
  }

  public DifferentialDriveOdometry getOdometry() {
    return m_odometry;
  }

}
