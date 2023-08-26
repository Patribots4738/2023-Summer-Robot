package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

public class LoadPath {
  
  private PathPlannerTrajectory loadedPath;

  public LoadPath(String name, PathConstraints constraints) {
    loadedPath = PathPlanner.loadPath(name, constraints);
  }

  public PathPlannerTrajectory getPath() {
    return loadedPath;
  }
}
