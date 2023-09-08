package frc.robot.auto;

import java.util.HashMap;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class EventMap {

    private HashMap<String, Command> eventMap;
    private FollowPathWithEvents followPath;
    private PathPlannerTrajectory basePath;
    private Drivetrain drivetrain;
    private Command command;

    // Only works for one path each instance
    public EventMap(PathPlannerTrajectory basePath, Drivetrain drivetrain) {
        this.eventMap = new HashMap<String, Command>();
        this.basePath = basePath;
        this.drivetrain = drivetrain;
        this.command = new FollowTrajectory(this.drivetrain, this.basePath, false).run();

        this.followPath = new FollowPathWithEvents(
            this.command,
            this.basePath.getMarkers(),
            this.eventMap
        );
    }

    public void addEvent(String event, Command command) {
        eventMap.put(event, command);
    }

    public Command getEvent(String event) {
        return eventMap.get(event);
    }

    public HashMap<String, Command> getEventMap() {
        return eventMap;
    }

    public FollowPathWithEvents getFollowPath() {
        return followPath;
    }

}
