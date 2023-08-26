package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PlacementConstants;
import frc.robot.subsystems.Claw;

public class SetClawSpeedPlacement extends CommandBase {
  private final Claw claw;
  private final int placementIndex;
  private final double totalPlacementTime;
  private Timer timer;


  public SetClawSpeedPlacement(Claw claw, int placementIndex) {
    // set the claw and placement position
    // add the claw as a requirement
    this.claw = claw;
    this.placementIndex = placementIndex;
    addRequirements(claw);
    
    timer = new Timer();
    timer.reset();
    // create a placement time variable for the claw to know how long to run
    totalPlacementTime = PlacementConstants.PLACEMENT_TIMES[placementIndex];
  }

  @Override
  public void initialize() {
    // set the speed of the claw to the placement position
    claw.setSpeed(PlacementConstants.PLACEMENT_SPEEDS_FRONT[placementIndex]);

    timer.start();
  }

  @Override
  public void execute() { }

  @Override
  public void end(boolean interrupted) {
    // Stop the claw by setting the speed to 0.0
    claw.setSpeed(0.0);
    // We are also going to stop the timer
    timer.stop();
  }

  @Override
  public boolean isFinished() {
    return timer.get() > totalPlacementTime;
  }

}
