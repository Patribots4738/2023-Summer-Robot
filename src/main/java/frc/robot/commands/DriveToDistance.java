package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class DriveToDistance extends PIDCommand {
  
  private final Drivetrain drivetrain;

  public DriveToDistance(Drivetrain drivetrain, double distance) {
    super(
      new PIDController(DrivetrainConstants.DRIVING_P, DrivetrainConstants.DRIVING_I, DrivetrainConstants.DRIVING_D),
      () -> drivetrain.getLeftEncoder().getPosition(),
      () -> distance,
      output -> drivetrain.run(output, 0)
      );

    getController().setTolerance(4);
    this.drivetrain = drivetrain;
  }

  @Override
  public void initialize() {
    // Drive to point
    drivetrain.resetEncoders();
    drivetrain.modeBreak();
  }

  @Override
  public void end(boolean interrupted) {
    // Stop motors
    drivetrain.run(0, 0);
  }

  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }

}