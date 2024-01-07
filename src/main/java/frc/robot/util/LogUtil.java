package frc.robot.util;

import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class LogUtil {
  public static void recordMetadata(String key, Object value) {
    if (value == null) {
      return;
    }
    StringLogEntry logEntry = new StringLogEntry(DataLogManager.getLog(), "/Metadata/" + key);
    logEntry.append(value.toString());
    logEntry.finish();
  }
}
