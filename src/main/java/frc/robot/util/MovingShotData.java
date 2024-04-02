package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * Represents data of a shooting on the move state
 *
 * <p>This class is a programming crime but it's for the sake of the Java GC Everything in this is
 * mutable to be more GC friendly, only 2 instances have to be created
 */
public class MovingShotData implements Sendable {
  public Translation2d virtualGoalLocation;
  public Rotation2d armAngle;
  public ShooterState shooterState;
  public double shotTime;
  public double distance;
  public int iterations;

  public MovingShotData(
      Translation2d virtualGoalLocation,
      Rotation2d armAngle,
      ShooterState shooterState,
      double shotTime,
      double distance,
      int iterations) {
    this.virtualGoalLocation = virtualGoalLocation;
    this.armAngle = armAngle;
    this.shooterState = shooterState;
    this.shotTime = shotTime;
    this.distance = distance;
    this.iterations = iterations;
  }

  public MovingShotData() {
    this(new Translation2d(), new Rotation2d(), new ShooterState(), 0.0, 0.0, 0);
  }

  public void replaceAll(MovingShotData other) {
    virtualGoalLocation = other.virtualGoalLocation;
    armAngle = other.armAngle;
    shooterState = other.shooterState;
    shotTime = other.shotTime;
    distance = other.distance;
    iterations = other.iterations;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleArrayProperty(
        "Virtual Goal Location",
        () -> new double[] {virtualGoalLocation.getX(), virtualGoalLocation.getY()},
        null);
    builder.addDoubleArrayProperty(
        "Shooter State",
        () -> new double[] {shooterState.topSpeed(), shooterState.bottomSpeed()},
        null);
    builder.addDoubleProperty("Arm Angle", () -> armAngle.getDegrees(), null);
    builder.addDoubleProperty("Shot Time", () -> shotTime, null);
    builder.addDoubleProperty("Distance", () -> distance, null);
    builder.addIntegerProperty("Iterations", () -> iterations, null);
  }
}
