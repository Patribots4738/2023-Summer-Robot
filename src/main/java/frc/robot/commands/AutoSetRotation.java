// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Pivot;

public class AutoSetRotation extends CommandBase {
  /** Creates a new BaseDrive. */
  private final Pivot pivot;
  private final int placementPosition;
  
  public AutoSetRotation(Pivot pivot, Claw claw, int placementPosition) {
    // set the pivot and placement position
    // add the pivot as a requirement
    Trigger whenFinished = new Trigger(() -> isFinished());
    whenFinished.onTrue(new AutoSetClawSpeed(claw, placementPosition));

    this.pivot = pivot;
    this.placementPosition = placementPosition;
    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // set the rotation of the pivot to the placement position
    pivot.setRotation(placementPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // start the next command for the claw to intake
  }

  @Override
  public boolean isFinished(){
    /*
     * return true if the pivot is at the placement position, given a deadband @see PivotConstants.PIVOT_DEADBAND_DEGREES
     */ 
    return Math.abs(pivot.getRotationDegrees() - placementPosition) < PivotConstants.PIVOT_DEADBAND_DEGREES;
  }
}
