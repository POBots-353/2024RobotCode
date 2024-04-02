package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.AutoShootConstants;
import frc.robot.Constants.FieldConstants;

public class AutoShootMathUtil {
  public static boolean isFacingSpeaker(Pose2d robotPose, Translation2d virtualGoalLocation) {
    if (Math.abs(robotPose.getRotation().getCos()) == 0.0) {
      return false;
    }
    // y = m(goalx) + b
    // b = roboty - m(robotx)
    double slope = Math.tan(robotPose.getRotation().getRadians());
    if (Double.isInfinite(slope) || Double.isNaN(slope)) {
      return false;
    }

    double b = robotPose.getY() - slope * robotPose.getX();

    double yIntersect = slope * virtualGoalLocation.getX() + b;

    double upperBound =
        virtualGoalLocation.getY()
            + (FieldConstants.speakerWidth - AutoShootConstants.speakerEdgeTolerance) / 2;
    double lowerBound =
        virtualGoalLocation.getY()
            - (FieldConstants.speakerWidth - AutoShootConstants.speakerEdgeTolerance) / 2;

    return yIntersect >= lowerBound && yIntersect <= upperBound;
  }
}
