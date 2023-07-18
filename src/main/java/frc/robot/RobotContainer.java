// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.BaseDrive;
import frc.robot.commands.ManualSetClawSpeed;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.PlacementConstants;
import frc.robot.commands.AutoSetPivotRotation;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pivot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Subsystems
  private final Drivetrain drivetrain = new Drivetrain();

  private final XboxController driverController = new XboxController(Constants.DRIVER_CONTROLLER_PORT);

  private final Pivot pivot = new Pivot();
  private final Claw claw = new Claw();

  private final BaseDrive baseDrive = new BaseDrive(
      drivetrain,
      () -> MathUtil.applyDeadband(driverController.getLeftY(), Constants.DRIVER_DEADBAND),
      () -> MathUtil.applyDeadband(driverController.getRightX(), Constants.DRIVER_DEADBAND)
  );

  private final ManualSetClawSpeed manualSetClawSpeed = new ManualSetClawSpeed(
    claw,
    () -> driverController.getLeftTriggerAxis() > 0 ? 
        driverController.getLeftTriggerAxis() :
        driverController.getRightTriggerAxis()
  );

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    drivetrain.setDefaultCommand(baseDrive);
    // claw.setDefaultCommand(manualSetClawSpeed);
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // create triggers for each button used

    Trigger y = new Trigger(() -> driverController.getYButton());
    Trigger b = new Trigger(() -> driverController.getBButton());
    Trigger a = new Trigger(() -> driverController.getAButton());
    Trigger x = new Trigger(() -> driverController.getXButton());

    Trigger leftTrigger = new Trigger(() -> driverController.getLeftTriggerAxis() > 0.2);
    Trigger rightTrigger = new Trigger(() -> driverController.getRightTriggerAxis() > 0.2);

    // High
    y.onTrue(movePivotAndClaw(PlacementConstants.HIGH_INDEX));
    // Middle
    b.onTrue(movePivotAndClaw(PlacementConstants.MID_INDEX));
    // Low
    a.onTrue(movePivotAndClaw(PlacementConstants.LOW_INDEX));
    // Reset
    x.onTrue(movePivotAndClaw(PlacementConstants.RESET_INDEX));

    /*
     * When A button is pressed, we want to set the pivot to a certain angle
     * Then, we want to wait until that angle is reached by the pivot
     * Finally, we want to move the claw at a certain speed
     */

    leftTrigger.whileTrue(Commands.run(() -> claw.setSpeed(driverController.getLeftTriggerAxis())))
                .onFalse(new InstantCommand(() -> claw.setSpeed(0)));

    rightTrigger.whileTrue(Commands.run(() -> claw.setSpeed(-driverController.getRightTriggerAxis())))
                .onFalse(new InstantCommand(() -> claw.setSpeed(0)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }

  public Command movePivotAndClaw(int index){
    return new AutoSetPivotRotation(pivot, claw, index);
    // Uncomment below to enable/disable the claw
    // .andThen(new InstantCommand(() -> claw.setSpeed(PlacementConstants.PLACEMENT_SPEEDS[index]))
    // .andThen(new WaitCommand(PlacementConstants.PLACEMENT_TIMES[index])))
    // .andThen(new InstantCommand(() -> claw.setSpeed(0)));
  }
}
