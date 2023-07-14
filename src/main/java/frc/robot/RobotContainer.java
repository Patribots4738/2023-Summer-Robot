// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.BaseDrive;
import frc.robot.commands.AutoSetRotation;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pivot;
import edu.wpi.first.wpilibj2.command.Command;
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
    Trigger aTrigger = new Trigger(() -> controller.getAButton());
    Trigger bTrigger = new Trigger(() -> controller.getBButton());
    Trigger xTrigger = new Trigger(() -> controller.getXButton());
    Trigger yTrigger = new Trigger(() -> controller.getYButton());

    // TODO: determine the placement positions for each button
    aTrigger.onTrue(new AutoSetRotation(pivot, claw, 0));
    bTrigger.onTrue(new AutoSetRotation(pivot, claw, 1));
    xTrigger.onTrue(new AutoSetRotation(pivot, claw,  2));
    yTrigger.onTrue(new AutoSetRotation(pivot, claw, 3));

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
