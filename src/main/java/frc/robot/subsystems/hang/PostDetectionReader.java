package frc.robot.subsystems.hang;

import static frc.robot.subsystems.hang.PostAlignConstants.*;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.networktables.TimestampedBoolean;
import org.littletonrobotics.junction.Logger;

/**
 * Reads PostDetection from NetworkTables (coprocessor RealSense). Single subscriber set per robot
 * process.
 */
public final class PostDetectionReader {

  private static PostDetectionReader instance;

  private final BooleanSubscriber postDetectedSub;
  private final DoubleSubscriber postLateralSub;
  private final DoubleSubscriber postDepthSub;

  private PostDetectionReader() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(kNetworkTableName);
    postDetectedSub = table.getBooleanTopic(kEntryDetected).subscribe(false);
    postLateralSub = table.getDoubleTopic(kEntryLateral).subscribe(0.0);
    postDepthSub = table.getDoubleTopic(kEntryDepth).subscribe(0.0);
  } // End PostDetectionReader Constructor

  /** Shared reader so button triggers do not create duplicate NT subscribers. */
  public static synchronized PostDetectionReader getInstance() {
    if (instance == null) {
      instance = new PostDetectionReader();
    }
    return instance;
  } // End getInstance

  /** True if post_detected has a published timestamp and updates are recent. */
  public boolean isPublishing() {
    TimestampedBoolean atomic = postDetectedSub.getAtomic();
    if (atomic.timestamp == 0) {
      return false;
    }
    long now = NetworkTablesJNI.now();
    return (now - postDetectedSub.getLastChange()) <= kPostDetectionMaxAgeMicros;
  } // End isPublishing

  public boolean isPostDetected() {
    return postDetectedSub.get();
  } // End isPostDetected

  public double getPostLateralMeters() {
    return postLateralSub.get();
  } // End getPostLateralMeters

  public double getPostDepthMeters() {
    return postDepthSub.get();
  } // End getPostDepthMeters

  /** Log current values for tuning. */
  public void logOutputs() {
    Logger.recordOutput("PostAlign/NT/post_detected", isPostDetected());
    Logger.recordOutput("PostAlign/NT/post_lateral_m", getPostLateralMeters());
    Logger.recordOutput("PostAlign/NT/post_depth_m", getPostDepthMeters());
    Logger.recordOutput("PostAlign/NT/stream_live", isPublishing());
  } // End logOutputs
} // End PostDetectionReader class
