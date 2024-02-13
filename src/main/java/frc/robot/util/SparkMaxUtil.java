package frc.robot.util;

import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;

public class SparkMaxUtil {
  public static final int disableFramePeriod = 65535;
  private static final PeriodicFrame[] peridicFrames = PeriodicFrame.values();

  public static void configureFollower(CANSparkMax follower) {
    for (PeriodicFrame frame : peridicFrames) {
      follower.setPeriodicFramePeriod(frame, disableFramePeriod);
    }
  }
}
