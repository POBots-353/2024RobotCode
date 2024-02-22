package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import java.util.Optional;

public class AllianceUtil {
  public static boolean isRedAlliance() {
    Optional<Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isPresent()) {
      return alliance.get() == Alliance.Red;
    } else {
      return false;
    }
  }

  public static Rotation2d getZeroRotation() {
    if (isRedAlliance()) {
      return Rotation2d.fromDegrees(180.0);
    } else {
      return Rotation2d.fromDegrees(0.0);
    }
  }

  public static Pose2d getSpeakerPose() {
    return isRedAlliance() ? FieldConstants.speakerRedAlliance : FieldConstants.speakerBlueAlliance;
  }
}
