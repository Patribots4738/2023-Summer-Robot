package frc.robot.auto;

import java.util.HashMap;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.limelight.LimelightBase;

public class AutoProgram {
    Drivetrain drivetrain;
    LimelightBase vision;
    private HashMap<String, Command> auto;

    public AutoProgram() {
        this.drivetrain = Drivetrain.getInstance();
        this.vision = Vision.getInstance();

        init();
    }

    private void init() {
        this.auto = new HashMap<String, Command>();

        Trajectories.autoSpeed = 2.5;
        auto.put("DEFAULT", Commands.sequence(Trajectories.startScoringPoint(PivotConstants.HIGH_INDEX),
                Trajectories.resetOdometry(false)));

        // TODO: Add more autos

        Set<String> array = auto.keySet();

        String[] autoArray = new String[array.size()];

        // TODO: check if this works --> if not, use a for loop
        array.toArray(autoArray);

        // TODO: add autos to Dashboard
    }

    public Command getAutoCommand(String name) {
        String selectedAuto = Dashboard.getSelectedAuto();

        if (selectedAuto == null) {
            return auto.get("DEFAULT");
        }

        return auto.get(selectedAuto);
    }

    private Pose2d inverseRoatation(Pose2d pose) {
        return new Pose2d(pose.getTranslation(), new Rotation2d(pose.getRotation().getRadians() + Math.PI));
    }
}
