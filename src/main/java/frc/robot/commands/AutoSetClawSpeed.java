package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.Claw;

public class AutoSetClawSpeed extends CommandBase{
  private final Claw claw;
  private final int placementPosition;

  //TODO: Add speeds for each position -> through testing

  public AutoSetClawSpeed(Claw claw, int placementPosition) {
    // set the claw and placement position
    // add the claw as a requirement
    this.claw = claw;
    this.placementPosition = placementPosition;
    addRequirements(claw);
  }

  @Override
  public void initialize() {
    // set the speed of the claw to the placement position
    claw.setSpeed(ClawConstants.PLACEMENT_SPEEDS[placementPosition]);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    // stop the claw by setting the speed to 0.0
    claw.setSpeed(0.0);
  }

  @Override
  public boolean isFinished(){
    // TODO: return true if after a certain amount of time. (dont need to do just yet)
    return false;
  }

}
