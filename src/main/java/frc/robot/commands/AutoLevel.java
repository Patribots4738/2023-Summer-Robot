package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.subsystems.Drivetrain;

public class AutoLevel extends InstantCommand{

    Timer elapsedTime;

    Command driveForward;
    Command driveBackward;

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

      System.out.printf("Elapsed Time: %.1f, Full output: %.2f\n", elapsedTime, ((AlignmentConstants.CHARGE_PAD_CORRECTION_P * tilt)/(elapsedTime/16)));

      if (tilt > Math.toRadians(7)) {
        swerve.drive(
            MathUtil.clamp(((AlignmentConstants.CHARGE_PAD_CORRECTION_P * tilt)/(elapsedTime/(DriverStation.isAutonomous() ? 10 : 20))), 0.055, 0.20),
            0, 
            0, 
            true, false);
      }
      else if (tilt < -Math.toRadians(7)) {
        swerve.drive(
            MathUtil.clamp(((AlignmentConstants.CHARGE_PAD_CORRECTION_P * tilt)/(elapsedTime/(DriverStation.isAutonomous() ? 10 : 20))), -0.20, -0.055),
            0, 
            0, 
            true, false);
      }
      else {
        swerve.setWheelsUp();
      }
     */
    public AutoLevel(){
        this.elapsedTime = new Timer();
        
        this.driveForward = new InstantCommand(() -> Drivetrain.getInstance().drive(0.2, 0));
        this.driveBackward = new InstantCommand(() -> Drivetrain.getInstance().drive(-0.2, 0));

        addRequirements(Drivetrain.getInstance());
    }

    @Override
    public void execute() {
        elapsedTime.start();
        double tilt = Drivetrain.getInstance().getGyroAngleDegrees();

        boolean isTilted = tilt > 7 || tilt < -7;???????

        ConditionalCommand repeatCommand = new ConditionalCommand(
            this.driveForward.until(() -> isTilted),
            new ConditionalCommand(
                this.driveBackward.until(() -> isTilted), 
                StopDrive.getCommand(), 
                () -> tilt < -7),
            () -> tilt > 7 );

        repeatCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
