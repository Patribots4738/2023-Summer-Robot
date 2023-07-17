package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;


public class ManualSetClawSpeed extends CommandBase {
  private final Claw claw;
  private DoubleSupplier desiredSpeed;


  public ManualSetClawSpeed(Claw claw, DoubleSupplier desiredSpeed) {
    // set the claw and placement position
    // add the claw as a requirement
    this.claw = claw;
    this.desiredSpeed = desiredSpeed;
    addRequirements(claw);
  }

   // Called when the command is initially scheduled.
   @Override
   public void initialize() {}
 
   // Called every time the scheduler runs while the command is scheduled.
   @Override
   public void execute() {
     claw.setSpeed(desiredSpeed.getAsDouble());
   }
 
   // Called once the command ends or is interrupted.
   @Override
   public void end(boolean interrupted) {
     claw.setSpeed(0);
   }
 
   // Returns true when the command should end.
   // We never want this command to end, so it always returns false
   @Override
   public boolean isFinished() {
     return false;
   }

}
