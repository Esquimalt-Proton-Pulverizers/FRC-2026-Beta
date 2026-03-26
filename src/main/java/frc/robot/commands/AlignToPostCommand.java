package frc.robot.commands;

import static frc.robot.subsystems.hang.PostAlignConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hang.PostDetectionReader;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Robot-centric alignment to the hang post using PostDetection. Backs on -X until detected, centers
 * lateral, then closes depth. Sets success flag for the caller.
 */
public class AlignToPostCommand extends Command {

  private enum Phase {
    SEARCH,
    LATERAL,
    DEPTH,
    DONE
  } // End Phase enum

  private final Drive drive;
  private final PostDetectionReader reader;
  private final AtomicBoolean successHolder;

  private Phase phase = Phase.SEARCH;
  private double commandStartTime;
  private double detectTime;
  private boolean everDetected;

  public AlignToPostCommand(
      Drive drive, PostDetectionReader reader, AtomicBoolean successHolder) {
    this.drive = drive;
    this.reader = reader;
    this.successHolder = successHolder;
    addRequirements(drive);
  } // End AlignToPostCommand Constructor

  @Override
  public void initialize() {
    phase = Phase.SEARCH;
    commandStartTime = Timer.getFPGATimestamp();
    detectTime = 0.0;
    everDetected = false;
    successHolder.set(false);
  } // End initialize

  @Override
  public void execute() {
    reader.logOutputs();

    double now = Timer.getFPGATimestamp();
    double elapsed = now - commandStartTime;
    if (!everDetected && elapsed >= kSearchTimeoutSeconds) {
      phase = Phase.DONE;
      successHolder.set(false);
      return;
    }
    if (everDetected && (now - detectTime) >= kAlignTimeoutSeconds) {
      phase = Phase.DONE;
      successHolder.set(false);
      return;
    }

    boolean detected = reader.isPostDetected();
    if (detected) {
      if (!everDetected) {
        everDetected = true;
        detectTime = now;
      }
    }

    switch (phase) {
      case SEARCH:
        if (detected) {
          phase = Phase.LATERAL;
        } else {
          drive.runVelocity(new ChassisSpeeds(kSearchVxMetersPerSec, 0.0, 0.0));
        }
        break;
      case LATERAL:
        if (!detected) {
          phase = Phase.SEARCH;
          drive.runVelocity(new ChassisSpeeds(kSearchVxMetersPerSec, 0.0, 0.0));
          break;
        }
        {
          // RealSense +X = image right; with boresight -X toward post, map to robot Y (tune bias).
          // Convert camera lateral to hook lateral by applying the camera Y offset from the hook.
          double lateralError =
              -reader.getPostLateralMeters() - kCameraOffsetFromHookYMeters + kLateralErrorBiasMeters;
          if (Math.abs(lateralError) <= kLateralToleranceMeters) {
            phase = Phase.DEPTH;
            drive.runVelocity(new ChassisSpeeds(0.0, 0.0, 0.0));
          } else {
            double vy = MathUtil.clamp(
                kLateralKp * lateralError, -kMaxAlignVyMetersPerSec, kMaxAlignVyMetersPerSec);
            drive.runVelocity(new ChassisSpeeds(0.0, vy, 0.0));
          }
        }
        break;
      case DEPTH:
        if (!detected) {
          phase = Phase.SEARCH;
          drive.runVelocity(new ChassisSpeeds(kSearchVxMetersPerSec, 0.0, 0.0));
          break;
        }
        {
          double depthError = reader.getPostDepthMeters() - kTargetDepthMeters;
          if (Math.abs(depthError) <= kDepthToleranceMeters) {
            phase = Phase.DONE;
            successHolder.set(true);
            drive.runVelocity(new ChassisSpeeds(0.0, 0.0, 0.0));
          } else {
            // Too far: positive depthError → more -X (negative vx).
            double vx = MathUtil.clamp(
                -kDepthKp * depthError,
                -kMaxAlignVxMetersPerSec,
                  kMaxAlignVxMetersPerSec);
            drive.runVelocity(new ChassisSpeeds(vx, 0.0, 0.0));
          }
        }
        break;
      case DONE:
      default:
        drive.runVelocity(new ChassisSpeeds(0.0, 0.0, 0.0));
        break;
    }
  } // End execute

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    if (interrupted) {
      successHolder.set(false);
    }
  } // End end

  @Override
  public boolean isFinished() {
    return phase == Phase.DONE;
  } // End isFinished
} // End AlignToPostCommand class
