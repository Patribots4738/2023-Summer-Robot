// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.BaseDrive;
import frc.robot.Constants.PlacementConstants;
import frc.robot.commands.AutoSetPivotRotation;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pivot;
import io.github.oblarg.oblog.Loggable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
public class RobotContainer implements Loggable{
  // The robot's subsystems and commands are defined here...

  // Subsystems
  private final Drivetrain drivetrain = new Drivetrain();

  private final XboxController driverController = new XboxController(Constants.DRIVER_CONTROLLER_PORT);
  private final XboxController operatorController = new XboxController(Constants.OPERATOR_CONTROLLER_PORT);

  private final Pivot pivot = new Pivot();
  private final Claw claw = new Claw();
  private boolean shootingBackwards;

  /*
    () -> Explantion:
      We need to give a supplier to the BaseDrive constructor. The supplier
      will be called every time the command is scheduled. We want to give the
      supplier the controller axis, without calling the controller axis once.
      This is why we use () ->
  */
  private final BaseDrive baseDrive = new BaseDrive(
      drivetrain,
      () -> MathUtil.applyDeadband(driverController.getLeftY(), Constants.DRIVER_DEADBAND_FORWARD),
      () -> MathUtil.applyDeadband(driverController.getRightX() * 0.8, Constants.DRIVER_DEADBAND_TURN)
  );

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    drivetrain.setDefaultCommand(baseDrive);
    // claw.setDefaultCommand(manualSetClawSpeed);
    configureButtonBindings();

    RobotContainer.incinerateMotors();
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

    // Trigger aDrive = new Trigger(() -> driverController.getAButton());
    // aDrive.onTrue(Commands.run(()-> drivetrain.setDefaultCommand(baseDriveSlow))).onFalse(Commands.run(() ->drivetrain.setDefaultCommand(baseDrive)));

    Trigger y = new Trigger(() -> operatorController.getYButton());
    Trigger b = new Trigger(() -> operatorController.getBButton());
    Trigger a = new Trigger(() -> operatorController.getAButton());
    Trigger x = new Trigger(() -> operatorController.getXButton());

    Trigger leftTrigger = new Trigger(() -> operatorController.getLeftTriggerAxis() > 0.2);
    Trigger leftBumper = new Trigger(() -> operatorController.getLeftBumper());
    Trigger rightTrigger = new Trigger(() -> operatorController.getRightTriggerAxis() > 0.2);

    Trigger rightStickPressed = new Trigger(() -> operatorController.getRightStickButton());

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

    leftTrigger.whileTrue(Commands.run(() -> claw.setSpeed(operatorController.getLeftTriggerAxis()/2)));
                // .onFalse(new InstantCommand(() -> claw.setSpeed(0)));

    leftBumper.onTrue(new InstantCommand(() -> claw.setSpeed(0)));

    rightTrigger.whileTrue(Commands.run(() -> claw.setSpeed(-operatorController.getRightTriggerAxis())))
                .onFalse(new InstantCommand(() -> claw.setSpeed(0)));

    rightStickPressed.onTrue(new InstantCommand(() -> shootingBackwards = !shootingBackwards));
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

  public Command movePivotAndClaw(int index) {
    return new AutoSetPivotRotation(pivot, claw, shootingBackwards ? PlacementConstants.PLACEMENT_POSITIONS_BACK[index] : PlacementConstants.PLACEMENT_POSITIONS_FRONT[index])
    .andThen(new InstantCommand(() -> claw.setSpeed(shootingBackwards ? PlacementConstants.PLACEMENT_SPEEDS_BACK[index] : PlacementConstants.PLACEMENT_SPEEDS_FRONT[index]))
    .andThen(new WaitCommand(PlacementConstants.PLACEMENT_TIMES[index])))
    .andThen(new InstantCommand(() -> claw.setSpeed(0)))
    .andThen(new WaitCommand(2))
    .andThen(new AutoSetPivotRotation(pivot, claw, PlacementConstants.RESET_PLACEMENT)); 
  }

  /**
   * Run burnFlash() for all controllers initialized. burnFlash() stops comms w/deivce for 200ms or more.
   * Might include calls from before method was called or calls from after. 
   * Too risky so we do this to burn everything in sync
   * to avoid accidentally stopping messages from getting to the device
   */
  public static void incinerateMotors() {
    Timer.delay(0.25);
    for (CANSparkMax spark : Constants.SPARK_LIST) {
      spark.burnFlash();
      Timer.delay(0.005);
    }
    Timer.delay(0.25);
    // System.out.println("\n\nAll motor flashes burnt\n\n");
  }
}
