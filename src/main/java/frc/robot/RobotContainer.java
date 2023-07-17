// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoSetPivotRoation;
import frc.robot.commands.BaseDrive;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pivot;

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

  private final XboxController controller = new XboxController(Constants.DRIVER_CONTROLLER_PORT);

  private final Pivot pivot = new Pivot();
  private final Claw claw = new Claw();

  private final BaseDrive baseDrive = new BaseDrive(
      drivetrain,
      () -> MathUtil.applyDeadband(controller.getLeftY(), Constants.DRIVER_DEADBAND),
      () -> MathUtil.applyDeadband(controller.getRightX(), Constants.DRIVER_DEADBAND)
  );

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    drivetrain.setDefaultCommand(baseDrive);
    pivot.setDefaultCommand(Commands.run(() -> pivot.periodic()));
    claw.setDefaultCommand(Commands.run(() -> claw.periodic()));
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
    Trigger y = new Trigger(() -> controller.getYButton());
    Trigger b = new Trigger(() -> controller.getBButton());
    Trigger a = new Trigger(() -> controller.getAButton());
    Trigger x = new Trigger(() -> controller.getXButton());
    Trigger rTrigger = new Trigger(() -> controller.getLeftTriggerAxis() > 0.4);
    Trigger lTrigger = new Trigger(() -> controller.getRightTriggerAxis() > 0.4);

    // High
    y.onTrue(new AutoSetPivotRoation(pivot, claw, 0)); 
    // Middle
    b.onTrue(new AutoSetPivotRoation(pivot, claw, 1)); 
    // Low
    a.onTrue(new AutoSetPivotRoation(pivot, claw, 2)); 
    // Reset
    x.onTrue(new AutoSetPivotRoation(pivot, claw,  3)); 
    // Intake 
    lTrigger.onTrue(Commands.run(() -> claw.setSpeed(0.3) ));
    // Outtake
    rTrigger.onTrue(Commands.run(() -> claw.setSpeed(-0.4)));
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
}
