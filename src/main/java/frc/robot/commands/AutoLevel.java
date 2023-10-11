package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.Constants.AlignmentConstants;
import frc.robot.subsystems.Drivetrain;

public class AutoLevel extends InstantCommand{

    Timer elapsedTime;

    Command driveForward;
    Command driveBackward;

    boolean tiltedLeft;
    boolean tiltedRight;


    /*
     * double elapsedTime = Timer.getFPGATimestamp() - startedChargePad;
      // boolean setWheelsUp = false;
      double tilt = 0;

      // If our heading is within -45 to 45 degrees or within -135 and -180 or within 135 to 180, use the pitch
      // Otherwise, use the roll
      if (-45 < swerve.getYaw().getDegrees() && swerve.getYaw().getDegrees() < 45) {
        tilt = -swerve.getPitch().getRadians();
      }
      else if (-180 < swerve.getYaw().getDegrees() && swerve.getYaw().getDegrees() < -135 ||
          135 < swerve.getYaw().getDegrees() && swerve.getYaw().getDegrees() < 180) 
      {
        tilt = swerve.getPitch().getRadians();
      }
      else if (-135 < swerve.getYaw().getDegrees() && swerve.getYaw().getDegrees() < -45) {
        tilt = swerve.getRoll().getRadians();
      }
      else if (45 < swerve.getYaw().getDegrees() && swerve.getYaw().getDegrees() < 135) 
      {
        tilt = -swerve.getRoll().getRadians();
      }
     */
    public AutoLevel(){
        this.elapsedTime = new Timer();
        
        addRequirements(Drivetrain.getInstance());
    }

    // TODO: Do we need to use pitch and roll as shown above?
    
    @Override
    public void execute() {
        elapsedTime.start();
        double tilt = Drivetrain.getInstance().getGyroAngleDegrees();

        this.tiltedLeft = tilt < 7;
        this.tiltedRight = tilt > -7;

        this.driveForward = new InstantCommand(
            () -> Drivetrain.getInstance().drive(
                MathUtil.clamp(
                    ((AlignmentConstants.CHARGE_PAD_CORRECTION_P * tilt)
                     / (elapsedTime.get() /(DriverStation.isAutonomous() 
                        ? 10 : 20))), 
                    0.055, 
                    0.20), 
                    0));

        this.driveBackward = new InstantCommand(
            () -> Drivetrain.getInstance().drive(
                MathUtil.clamp(
                    ((AlignmentConstants.CHARGE_PAD_CORRECTION_P * tilt)
                     / (elapsedTime.get() / (DriverStation.isAutonomous() 
                        ? 10 : 20))), 
                        -0.20, 
                        -0.055),
                        0));

        ConditionalCommand repeatCommand = new ConditionalCommand(
            this.driveForward.until(() -> tiltedRight),
            new ConditionalCommand(
                this.driveBackward.until(() -> tiltedLeft), 
                StopDrive.getCommand(), 
                () -> tiltedRight),
            () -> tiltedLeft );

        repeatCommand.execute();
    }

    @Override
    public boolean isFinished() {
        return (!tiltedLeft && !tiltedRight);
    }

}
