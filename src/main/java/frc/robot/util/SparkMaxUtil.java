package frc.robot.util;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;

public class SparkMaxUtil {
  public static final int disableFramePeriod = 65535;

  public static void configureFollower(CANSparkMax follower) {
    for (PeriodicFrame frame : CANSparkLowLevel.PeriodicFrame.values()) {
      follower.setPeriodicFramePeriod(frame, disableFramePeriod);
    }
  }
}
