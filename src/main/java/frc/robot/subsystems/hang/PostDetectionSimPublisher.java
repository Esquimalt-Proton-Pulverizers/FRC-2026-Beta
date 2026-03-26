package frc.robot.subsystems.hang;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.FieldConstants;

/**
 * Simulation-only publisher for NetworkTables {@code PostDetection}. It fakes the RealSense output
 * based on known ladder post positions and the current simulated robot pose.
 */
public final class PostDetectionSimPublisher {

  private static final double kSimClippingDepthMeters = 0.5;
  private static final double kFovHalfAngleRad = Math.toRadians(40.0);

  private final BooleanPublisher postDetectedPub;
  private final DoublePublisher postLateralPub;
  private final DoublePublisher postDepthPub;

  // Two posts on each end of the field; alliance color is ignored for sim (we publish whichever
  // post is closest in camera space within FOV + depth).
  private final Translation2d[] postPositionsField;

  public PostDetectionSimPublisher() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(PostAlignConstants.kNetworkTableName);
    postDetectedPub = table.getBooleanTopic(PostAlignConstants.kEntryDetected).publish();
    postLateralPub = table.getDoubleTopic(PostAlignConstants.kEntryLateral).publish();
    postDepthPub = table.getDoubleTopic(PostAlignConstants.kEntryDepth).publish();

    double postXBlueM = Units.inchesToMeters(41.8125);
    double postYLeftBlueM = Units.inchesToMeters(165.093750);
    double postYRightBlueM = Units.inchesToMeters(129.843750);
    double postXRedM = FieldConstants.FIELD_LENGTH_M - postXBlueM;

    postPositionsField =
        new Translation2d[] {
          new Translation2d(postXBlueM, postYLeftBlueM),
          new Translation2d(postXBlueM, postYRightBlueM),
          new Translation2d(postXRedM, postYLeftBlueM),
          new Translation2d(postXRedM, postYRightBlueM)
        };
  } // End PostDetectionSimPublisher Constructor

  public void update(Pose2d robotPose) {
    Rotation2d robotHeading = robotPose.getRotation();
    double theta = robotHeading.getRadians();

    // Robot frame: +X forward, +Y left. Drive pose is treated as robot frame origin at chassis
    // center. Convert robot pose to camera field position.
    double robotHalfLengthMeters = Constants.Dimensions.FULL_LENGTH.in(Meters) / 2.0;
    double backEdgeXRobot = -robotHalfLengthMeters;

    double cameraXRobot = backEdgeXRobot + PostAlignConstants.kCameraOffsetFromBackEdgeXMeters;
    double cameraYRobot =
        HangConstants.kHookCenterYMeters + PostAlignConstants.kCameraOffsetFromHookYMeters;

    double cos = Math.cos(theta);
    double sin = Math.sin(theta);

    // +X robot unit in field.
    double robotXFieldX = cos;
    double robotXFieldY = sin;
    // +Y robot unit in field.
    double robotYFieldX = -sin;
    double robotYFieldY = cos;

    double cameraXField = robotPose.getX() + cameraXRobot * robotXFieldX + cameraYRobot * robotYFieldX;
    double cameraYField = robotPose.getY() + cameraXRobot * robotXFieldY + cameraYRobot * robotYFieldY;

    // Camera optical axis points along robot -X.
    double camForwardX = -robotXFieldX;
    double camForwardY = -robotXFieldY;
    // Camera +X (image right) aligns with camera right; in this mounting it's robot +Y.
    double camRightX = robotYFieldX;
    double camRightY = robotYFieldY;

    boolean detected = false;
    double bestDepth = 0.0;
    double bestLateral = 0.0;
    double bestRange = Double.POSITIVE_INFINITY;

    for (Translation2d postPos : postPositionsField) {
      double dx = postPos.getX() - cameraXField;
      double dy = postPos.getY() - cameraYField;

      // Depth is along camera forward (+Z in the RealSense frame).
      double depth = dx * camForwardX + dy * camForwardY;
      if (depth <= 0.0 || depth > kSimClippingDepthMeters) {
        continue;
      }

      // Lateral is along camera right (+X in the RealSense frame).
      double lateral = dx * camRightX + dy * camRightY;
      double angle = Math.atan2(lateral, depth);
      if (Math.abs(angle) > kFovHalfAngleRad) {
        continue;
      }

      double range = Math.hypot(depth, lateral);
      if (range < bestRange) {
        bestRange = range;
        bestDepth = depth;
        bestLateral = lateral;
        detected = true;
      }
    }

    postDetectedPub.set(detected);
    postLateralPub.set(detected ? bestLateral : 0.0);
    postDepthPub.set(detected ? bestDepth : 0.0);
  } // End update
} // End PostDetectionSimPublisher class

