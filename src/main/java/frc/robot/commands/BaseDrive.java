// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class BaseDrive extends CommandBase {
    /** Creates a new BaseDrive. */
    private final Drivetrain drive;
    private DoubleSupplier turn;
    private DoubleSupplier forward;

    public BaseDrive(Drivetrain drive, DoubleSupplier forward, DoubleSupplier turn) {
        this.drive = drive;
        this.forward = forward;
        this.turn = turn;
        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        drive.drive(forward.getAsDouble() * DrivetrainConstants.DRIVING_SPEED_MULTIPLIER,
                turn.getAsDouble() * DrivetrainConstants.DRIVING_SPEED_MULTIPLIER);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        StopDrive stop = new StopDrive();
        stop.execute();
    }

    // Returns true when the command should end.
    // We never want this command to end, so it always returns false
    @Override
    public boolean isFinished() {
        return false;
    }
}
