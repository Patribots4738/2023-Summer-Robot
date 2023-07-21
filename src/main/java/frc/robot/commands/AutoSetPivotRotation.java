// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Pivot;

public class AutoSetPivotRotation extends CommandBase {
    /** Creates a new BaseDrive. */
    private final Pivot pivot;
    private double placementPosition;
    public boolean backwards;


    public AutoSetPivotRotation(Pivot pivot, double placementPosition) {
        // set the pivot and placement position
        // add the pivot as a requirement

        this.pivot = pivot;
        this.placementPosition = placementPosition;
        addRequirements(pivot);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        /*
         * set the pivot's rotation to the placement position given in the array
         * 
         * @see PivotConstants.PLACEMENT_ROTATION_POSITIONS
         * 
         * The placement position is the index of the array, so we can use that to get
         * the value from the array.
         */
        // System.out.println("Setting desired rotation to " + PlacementConstants.PLACEMENT_POSITIONS[placementIndex]);
        pivot.setDesiredRotation(placementPosition);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // start the next command for the claw to intake
    }

    @Override
    public boolean isFinished() {
        /*
         * return true if the pivot is within the deadband of the placement position
         * given in the array @see PivotConstants.PLACEMENT_ROTATION_POSITIONS.
         * 
         * To determine if the pivot is within the deadband, we can use the subtraction
         * of the pivot's current rotation and the placement position. If the absolute
         * value of
         * the difference is less than the deadband, then the motors should stop.
         */
        // System.out.println("Current rotation: " + pivot.getEncoderPositionDegrees() + 
        //                     " Desired Rotation " + PlacementConstants.PLACEMENT_POSITIONS[placementIndex]
        //     );
        return Math.abs(pivot.getEncoderPositionDegrees()
                - placementPosition) < PivotConstants.PIVOT_DEADBAND_DEGREES;
    }
}
