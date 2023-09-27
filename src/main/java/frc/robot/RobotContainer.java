// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.BaseDrive;
import frc.robot.commands.DriveToPoint;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pivot;
import io.github.oblarg.oblog.Loggable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.POVButton;
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
public class RobotContainer implements Loggable {
    // The robot's subsystems and commands are defined here...

    // Subsystems
    private final Drivetrain drivetrain = new Drivetrain();

    private final XboxController driverController = new XboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);
    private final XboxController operatorController = new XboxController(ControllerConstants.OPERATOR_CONTROLLER_PORT);

    private final Pivot pivot = new Pivot();
    private final Claw claw = new Claw();
    private boolean shootingBackwards;

    /*
     * () -> Explantion:
     * We need to give a supplier to the BaseDrive constructor. The supplier
     * will be called every time the command is scheduled. We want to give the
     * supplier the controller axis, without calling the controller axis once.
     * This is why we use () ->
     */
    private final BaseDrive baseDrive = new BaseDrive(
            drivetrain,
            () -> MathUtil.applyDeadband(driverController.getLeftY(), ControllerConstants.DRIVER_DEADBAND_FORWARD),
            () -> MathUtil.applyDeadband(driverController.getRightX() * 0.8, ControllerConstants.DRIVER_DEADBAND_TURN));

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
        // aDrive.onTrue(Commands.run(()->
        // drivetrain.setDefaultCommand(baseDriveSlow))).onFalse(Commands.run(()
        // ->drivetrain.setDefaultCommand(baseDrive)));

        /*
         * a = LOW / right bumper option
         * b = MID / right bumper option
         * y = HIGH / right bumper option
         * x = RESET ARM
         * 
         * left trigger = claw intake
         * right trigger = claw outtake
         * 
         * left bumper = claw stop
         * right bumper = outake and reset arm
         * 
         * pov 0 = ---
         * pov 90 = forward shooting - set
         * pov 180 = ---
         * pov 270 = backwards shooting - set
         * 
         */

        // LOW
        Trigger aPress = new Trigger(() -> operatorController.getAButton());
        // MID
        Trigger bPress = new Trigger(() -> operatorController.getBButton());
        // HIGH
        Trigger yPress = new Trigger(() -> operatorController.getYButton());
        // RESET
        Trigger xPress = new Trigger(() -> operatorController.getXButton());
        // POV
        POVButton POVRight = new POVButton(operatorController, 90);
        POVButton POVLeft = new POVButton(operatorController, 270);
        // CLAW INTAKE
        Trigger leftTrigger = new Trigger(() -> operatorController.getLeftTriggerAxis() > 0.5);
        // CLAW OUTTAKE
        Trigger rightTrigger = new Trigger(() -> operatorController.getRightTriggerAxis() > 0.5);
        // CLAW STOP
        Trigger leftBumper = new Trigger(() -> operatorController.getLeftBumper());
        // OUTTAKE AND RESET ARM
        Trigger rightBumper = new Trigger(() -> operatorController.getRightBumper());

        // CLAW INTAKE POSITIONS
        aPress.onTrue(new InstantCommand(() -> pivot.setArmLow(shootingBackwards)));
        bPress.onTrue(new InstantCommand(() -> pivot.setArmMid(shootingBackwards)));
        yPress.onTrue(new InstantCommand(() -> pivot.setArmHigh(shootingBackwards)));
        xPress.onTrue(new InstantCommand(() -> pivot.setArmReset()));

        // CLAW BACKWARDS OR FORWARDS
        POVRight.onTrue((new InstantCommand(() -> shootingBackwards = true)));
        POVLeft.onTrue((new InstantCommand(() -> shootingBackwards = false)));

        // TODO: set speed values to constant
        leftTrigger.onTrue(new InstantCommand(() -> claw.setSpeed(0.5)));
        rightTrigger.onTrue(new InstantCommand(() -> claw.setSpeed(-0.5)));

        // CLAW STOP
        leftBumper.onTrue(new InstantCommand(() -> claw.setSpeed(0)));

        // CLAW AUTO OUTTAKE
        // TODO: set speed values to constant
        rightBumper.onTrue(new InstantCommand(() -> claw.setSpeed(-0.5))
                .andThen(new InstantCommand(() -> pivot.setArmReset())));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new DriveToPoint(
            drivetrain.getPose(),
            new Translation2d(1, 1),
            0.5,
          drivetrain);
    }

    /**
     * Run burnFlash() for all controllers initialized. burnFlash() stops comms
     * w/deivce for 200ms or more.
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
