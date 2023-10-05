// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.auto.AutoPrograms;
import frc.robot.subsystems.Drivetrain;
import io.github.oblarg.oblog.Logger;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command autonomousCommand;
    public Timer timer;
    private RobotContainer robotContainer;
    public static AutoPrograms autoPrograms = new AutoPrograms();

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();
        Logger.configureLoggingAndConfig(robotContainer, false);

    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and
     * test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        Shuffleboard.update();
        Logger.updateEntries();
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        timer = new Timer();
        autonomousCommand = autoPrograms.getAutonomousCommand();
        if(autonomousCommand != null) {
            autonomousCommand.schedule();
            timer.start();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        CommandScheduler.getInstance().run();
        if(timer.hasElapsed(14.75)){
            new RunCommand(() -> { 
                Drivetrain.getInstance().drive(0, 0); 
                Drivetrain.getInstance().modeBreak(); 
            }, Drivetrain.getInstance());
        }
    }

    @Override
    public void teleopInit() {
        timer = new Timer();
        timer.start();
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        CommandScheduler.getInstance().run();
        if(timer.hasElapsed(134.75)){
            new RunCommand(() -> {
                Drivetrain.getInstance().drive(0, 0);
                Drivetrain.getInstance().modeBreak();
            }, Drivetrain.getInstance());
        }
    }
    
}
