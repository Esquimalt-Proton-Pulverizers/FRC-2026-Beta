package frc.robot.subsystems.hang;

import edu.wpi.first.math.util.Units;

/** Constants for RealSense post alignment before hang (NetworkTables PostDetection). */
public final class PostAlignConstants {

  private PostAlignConstants() {}

  /** NT table published by the coprocessor. */
  public static final String kNetworkTableName = "PostDetection";

  public static final String kEntryDetected = "post_detected";
  public static final String kEntryLateral = "post_lateral";
  public static final String kEntryDepth = "post_depth";

  /** Max age of post_detected updates to treat the stream as live (microseconds, NT clock). */
  public static final long kPostDetectionMaxAgeMicros = 500_000L;

  /** Lateral alignment tolerance for the hook (meters). */
  public static final double kLateralToleranceMeters = 0.02;
  /** Depth alignment tolerance (meters). */
  public static final double kDepthToleranceMeters = 0.05;

  /** RealSense lateral offset from hook: +Y left (meters). */
  public static final double kCameraOffsetFromHookYMeters = Units.inchesToMeters(-2.5);

  /** RealSense depth offset from the robot back edge (meters). */
  public static final double kCameraOffsetFromBackEdgeXMeters = Units.inchesToMeters(0.5);

  /**
   * Extra lateral bias from camera frame to hook (meters). Tune sign on-robot; RealSense +X is
   * image right.
   */
  public static final double kLateralErrorBiasMeters = 0.0;

  /** Target camera-to-post distance for robot back edge touch (tune on robot). */
  public static final double kTargetDepthMeters = kCameraOffsetFromBackEdgeXMeters;

  /** Slow -X search when post not visible (robot backs toward ladder). */
  public static final double kSearchVxMetersPerSec = -0.25;

  /** Cap alignment speeds (m/s). */
  public static final double kMaxAlignVxMetersPerSec = 0.35;
  public static final double kMaxAlignVyMetersPerSec = 0.35;

  /** P gains for closing lateral and depth errors. */
  public static final double kLateralKp = 2.0;
  public static final double kDepthKp = 2.0;

  /** Optional final strafe +Y before STORED; 0 speed disables the phase. */
  public static final double kFinalInchVyMetersPerSec = 0.0;
  public static final double kFinalInchSeconds = 0.15;

  /** Stop searching after this long without detection. */
  public static final double kSearchTimeoutSeconds = 5.0;
  /** Stop alignment after this long once detected (safety). */
  public static final double kAlignTimeoutSeconds = 15.0;
} // End PostAlignConstants class
