// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pivot;

public class SetRotation extends CommandBase {
  /** Creates a new BaseDrive. */
  private final Pivot pivot;
  private final int placementPosition;
  
  public SetRotation(Pivot pivot, int placementPosition) {
    this.pivot = pivot;
    this.placementPosition = placementPosition;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pivot.setRotation(placementPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO: apply deadband for the range of rotations 
    if (pivot.getRotation() == placementPosition) { 
      // TODO: check if this is the correct way to end the command
      end(true); 
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // start the next command for the claw to intake
  }
}