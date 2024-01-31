package frc.robot.util;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import java.util.List;
import java.util.function.Supplier;
import me.nabdev.pathfinding.Pathfinder;
import me.nabdev.pathfinding.PathfinderBuilder;
import me.nabdev.pathfinding.structures.ImpossiblePathException;
import me.nabdev.pathfinding.utilities.FieldLoader.Field;

public class AStarPathfinder implements com.pathplanner.lib.pathfinding.Pathfinder {
  private final PathfinderBuilder pathBuilder =
      new PathfinderBuilder(Field.CRESCENDO_2024)
          .setRobotLength(Units.inchesToMeters(30.0))
          .setRobotWidth(Units.inchesToMeters(30.0));
  private final Pathfinder pathFinder = pathBuilder.build();

  private Supplier<Pose2d> poseSupplier;

  private PathPlannerPath currentPath;
  private PathConstraints currentConstraints;
  private GoalEndState currentEndState;

  private Translation2d startPosition = new Translation2d();
  private Translation2d goalPosition = new Translation2d();

  private boolean replanPath = false;

  public AStarPathfinder(Supplier<Pose2d> poseSupplier) {
    this.poseSupplier = poseSupplier;
  }

  @Override
  public boolean isNewPathAvailable() {
    return replanPath;
  }

  @Override
  public PathPlannerPath getCurrentPath(PathConstraints constraints, GoalEndState goalEndState) {
    boolean generatePath = replanPath;
    generatePath = !constraints.equals(currentConstraints) || generatePath;
    generatePath = !goalEndState.equals(currentEndState) || generatePath;

    if (generatePath) {
      currentConstraints = constraints;
      currentEndState = goalEndState;

      generateNewPath();

      if (currentPath != null) {
        replanPath = false;
      }
    }

    return currentPath;
  }

  @Override
  public void setStartPosition(Translation2d startPosition) {
    replanPath = true;
    this.startPosition = startPosition;
  }

  @Override
  public void setGoalPosition(Translation2d goalPosition) {
    replanPath = true;
    this.goalPosition = goalPosition;
  }

  @Override
  public void setDynamicObstacles(
      List<Pair<Translation2d, Translation2d>> obs, Translation2d currentRobotPos) {}

  private void generateNewPath() {
    TrajectoryConfig config =
        new TrajectoryConfig(
                currentConstraints.getMaxVelocityMps(),
                currentConstraints.getMaxAccelerationMpsSq())
            .setEndVelocity(currentEndState.getVelocity());

    Pose2d currentPose = poseSupplier.get();
    Pose2d startingPose = new Pose2d(startPosition, currentPose.getRotation());

    Trajectory trajectory;
    try {
      trajectory =
          pathFinder.generateTrajectory(
              startingPose, new Pose2d(goalPosition, currentEndState.getRotation()), config);
    } catch (ImpossiblePathException e) {
      currentPath = null;
      return;
    }

    List<PathPoint> pathPoints =
        trajectory.getStates().stream()
            .map(
                (Trajectory.State state) -> {
                  return new PathPoint(state.poseMeters.getTranslation());
                })
            .toList();

    PathPlannerPath newPath =
        PathPlannerPath.fromPathPoints(pathPoints, currentConstraints, currentEndState);

    currentPath = newPath;
  }
}
