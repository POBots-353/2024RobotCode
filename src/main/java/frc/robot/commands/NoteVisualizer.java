package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructArrayTopic;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructTopic;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Robot;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import monologue.Logged;

public class NoteVisualizer implements Logged {
  private static StructArrayPublisher<Pose3d> notePathPublisher;
  private static StructPublisher<Pose3d> shotNotePublisher;

  private static DoubleSupplier velocitySupplier = () -> 0.0;
  private static Supplier<Rotation2d> angleSupplier = Rotation2d::new;
  private static Supplier<Pose2d> robotPoseSupplier = Pose2d::new;

  public static void setSuppliers(
      Supplier<Pose2d> robotPose, Supplier<Rotation2d> angle, DoubleSupplier velocity) {
    robotPoseSupplier = robotPose;
    angleSupplier = angle;
    velocitySupplier = velocity;
  }

  private static Pose3d currentNotePose = new Pose3d();
  private static Pose3d lastNotePose = new Pose3d();

  private static List<Pose3d> pathPositions = new ArrayList<>();

  private static double zLaunchVelocity;
  private static int pathIndex = 0;

  private static final int visualizationStep = 1;
  private static final double pathDt = 0.020;

  public static void startPublishers() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("Robot");

    StructArrayTopic<Pose3d> notePathTopic = table.getStructArrayTopic("Note Path", Pose3d.struct);
    StructTopic<Pose3d> shotNoteTopic = table.getStructTopic("Shot Note", Pose3d.struct);

    notePathPublisher = notePathTopic.publish();
    shotNotePublisher = shotNoteTopic.publish();
  }

  public static Command shoot() {
    if (RobotBase.isReal()) {
      return Commands.none();
    }
    return new ScheduleCommand(
            Commands.defer(
                () -> {
                  generatePath();

                  return Commands.run(
                          () -> {
                            currentNotePose = pathPositions.get(pathIndex);
                            pathIndex++;

                            shotNotePublisher.set(currentNotePose);
                          })
                      .until(() -> pathIndex == pathPositions.size() - 1)
                      .finallyDo(
                          () -> {
                            pathIndex = 0;
                            notePathPublisher.set(new Pose3d[0]);
                          })
                      // Edge case with crashing because note starts off the field
                      .unless(() -> pathIndex >= pathPositions.size() - 1);
                },
                Set.of()))
        .ignoringDisable(true)
        .unless(Robot::isReal);
  }

  private static void generatePath() {
    if (RobotBase.isReal()) {
      return;
    }
    double g = 9.81;
    double linearVelocity =
        velocitySupplier.getAsDouble()
            * ShooterConstants.gearing
            * ShooterConstants.circumference
            / 60.0
            / 2.00;

    Rotation2d shootingAngle =
        angleSupplier.get().plus(Rotation2d.fromDegrees(90.0)).plus(Rotation2d.fromDegrees(30.0));
    Rotation2d armPosition =
        angleSupplier.get().plus(Rotation2d.fromDegrees(180.0)); // flipped over origin

    Rotation2d robot = robotPoseSupplier.get().getRotation();

    lastNotePose =
        new Pose3d(robotPoseSupplier.get())
            .plus(
                new Transform3d(
                    new Translation3d(
                        ArmConstants.armPivotToShooter * armPosition.getCos()
                            + ArmConstants.armPivotX,
                        0.0,
                        -ArmConstants.armPivotToShooter * armPosition.getSin()
                            + ArmConstants.armPivotZ),
                    new Rotation3d(0, shootingAngle.getRadians(), 0)));

    final double xVelocity = -linearVelocity * robot.getCos() * shootingAngle.getCos();
    final double yVelocity = -linearVelocity * robot.getSin() * shootingAngle.getCos();
    zLaunchVelocity = linearVelocity * shootingAngle.getSin();

    pathPositions = new ArrayList<>();
    pathPositions.add(lastNotePose);

    int timestep = 0;
    while (lastNotePose.getZ() > 0.0
        && lastNotePose.getX() > -0.12
        && lastNotePose.getX() < FieldConstants.fieldLength + 0.12
        && lastNotePose.getY() > -0.12
        && lastNotePose.getY() < FieldConstants.fieldWidth + 0.12) {
      currentNotePose =
          new Pose3d(
              new Translation3d(
                  lastNotePose.getX() + xVelocity * pathDt,
                  lastNotePose.getY() + yVelocity * pathDt,
                  lastNotePose.getZ() + zLaunchVelocity * pathDt),
              lastNotePose.getRotation());

      if (timestep % visualizationStep == 0) {
        pathPositions.add(currentNotePose);
      }

      timestep++;
      zLaunchVelocity -= g * pathDt;
      lastNotePose = currentNotePose;
    }
    notePathPublisher.set(pathPositions.toArray(Pose3d[]::new));
  }
}
