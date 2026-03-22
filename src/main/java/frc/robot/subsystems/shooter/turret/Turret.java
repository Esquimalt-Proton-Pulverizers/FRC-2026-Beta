package frc.robot.subsystems.shooter.turret;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.drive.Drive;
import static frc.robot.subsystems.shooter.turret.TurretConstants.*;

/** Turret subsystem: one motor with onboard position control, aimed at a hub angle. */
public class Turret extends SubsystemBase {

  private final TurretIO turretIO;
  private final TurretIO.TurretIOInputs turretInputs = new TurretIO.TurretIOInputs();

  private static final Rotation2d kBackInRobotFrame = Rotation2d.kPi;
  private Rotation2d hubAngleRelativeToRobot = kBackInRobotFrame;
  private double velocityFeedforwardRadPerSec = 0.0;

  /** When false, the Turret will automatically aim towards the hub */
  private BooleanSupplier manualOverrideSupplier = () -> false;
  /** When true (and not manual override), Turret aims at shot target; when false, holds current position. */
  private BooleanSupplier aimAtTargetSupplier = () -> false;
  private Drive drive;

  private double lastSmartDashboardTargetPosRad = Math.PI;

  public Turret(TurretIO io) {
    turretIO = io;

    SmartDashboard.putNumber("Turret/kP", kP);
    SmartDashboard.putNumber("Turret/kI", kI);
    SmartDashboard.putNumber("Turret/kD", kD);
    SmartDashboard.putNumber("Turret/TargetPositionDeg", 180.0);
  } // End Turret Constructor

  /** Set by RobotContainer so calculator does not overwrite Turret when operator is in manual override. */
  public void setManualOverrideSupplier(BooleanSupplier supplier) {
    manualOverrideSupplier = supplier != null ? supplier : () -> false;
  } // End setManualOverrideSupplier

  /** Set by RobotContainer so Turret can get Robot Pose. */
  public void setDrive(Drive drive) {
    this.drive = drive;
  } // End setDrive

  /** Set by RobotContainer so Turret only aims at target when e.g. ShootWhenReadyCommand is active. */
  public void setAimAtTargetSupplier(BooleanSupplier supplier) {
    aimAtTargetSupplier = supplier != null ? supplier : () -> true;
  } // End setAimAtTargetSupplier

  @Override
  public void periodic() {
    double targetPositionRad;

    // When not in manual override, aim at the hub only if aim-at-target is enabled (e.g. ShootWhenReadyCommand active); otherwise hold position
    if (DriverStation.isDisabled()) {
      velocityFeedforwardRadPerSec = 0.0;
      targetPositionRad = MathUtil.clamp(turretInputs.positionRads, kMinAngleRad, kMaxAngleRad);
    } else if (!manualOverrideSupplier.getAsBoolean()) {
      if (aimAtTargetSupplier.getAsBoolean()) {
        setHubAngleRelativeToRobot(ShooterCommands.getTurretAngleFromShot(drive));
        setVelocityFeedforwardRadPerSec(-drive.getFieldRelativeChassisSpeeds().omegaRadiansPerSecond);
        targetPositionRad = getClampedTurretSetpointRad();
      } else {
        setVelocityFeedforwardRadPerSec(0.0);
        targetPositionRad = MathUtil.clamp(turretInputs.positionRads, kMinAngleRad, kMaxAngleRad);
      }
    } else {
      double targetRobotFrameRad = Units.degreesToRadians(SmartDashboard.getNumber("Turret/TargetPositionDeg", 180.0));
      if (targetRobotFrameRad != lastSmartDashboardTargetPosRad) {
        setHubAngleRelativeToRobot(Rotation2d.fromRadians(targetRobotFrameRad));
      }

      lastSmartDashboardTargetPosRad = targetRobotFrameRad;
      velocityFeedforwardRadPerSec = 0.0;
      targetPositionRad = getClampedTurretSetpointRad();
    }

    turretIO.updateInputs(turretInputs);
    Logger.recordOutput("Subsystems/Shooter/Turret/Inputs/MotorConnected", turretInputs.motorConnected);
    Logger.recordOutput("Subsystems/Shooter/Turret/Inputs/TargetPositionDeg", Units.radiansToDegrees(targetPositionRad));
    Logger.recordOutput("Subsystems/Shooter/Turret/Inputs/PositionDeg", Units.radiansToDegrees(turretInputs.positionRads));

    Logger.recordOutput("Subsystems/Shooter/Turret/PositionDeg", getPosition().getDegrees());
    Logger.recordOutput("Subsystems/Shooter/Turret/RobotFrameDeg", getRobotFramePosition().getDegrees());
    Logger.recordOutput("Subsystems/Shooter/Turret/HubDeg", getHubAngleRelativeToRobot().getDegrees());
    Logger.recordOutput("Subsystems/Shooter/Turret/Inputs/VelocityDegPerSec", Units.radiansToDegrees(turretInputs.velocityRadsPerSec));
    Logger.recordOutput("Subsystems/Shooter/Turret/Inputs/AppliedVolts", turretInputs.appliedVolts);
    Logger.recordOutput("Subsystems/Shooter/Turret/Inputs/SupplyCurrentAmps", turretInputs.supplyCurrentAmps);

    if (DriverStation.isDisabled()) {
      turretIO.setTargetPosition(targetPositionRad, 0.0);
      return;
    }

    turretIO.setTargetPosition(targetPositionRad, velocityFeedforwardRadPerSec);
  } // End periodic

  /** Set velocity feedforward for spin compensation; e.g. -robot omega. */
  public void setVelocityFeedforwardRadPerSec(double radPerSec) {
    velocityFeedforwardRadPerSec = radPerSec;
  } // End setVelocityFeedforwardRadPerSec

  /** Resets the motors position to 0 */
  public void resetMotorEncoder() {
    turretIO.stop();
    turretIO.resetEncoder();
    setHubAngleRelativeToRobot(kBackInRobotFrame);
    SmartDashboard.putNumber("Turret/TargetPositionDeg", 180.0);
  } // End resetMotorEncoder

  /** Set the hub angle (robot frame: 0 = forward). Clamped to min/max in periodic. */
  public void setHubAngleRelativeToRobot(Rotation2d angle) {
    hubAngleRelativeToRobot = angle;
  } // End setHubAngleRelativeToRobot

  /** Step the target position in Turret frame. */
  public void stepPositionRad(double stepPositionRad) {
    double turretTargetRad =
        MathUtil.clamp(turretInputs.targetPositionRads + stepPositionRad, kMinAngleRad, kMaxAngleRad);
    setHubAngleRelativeToRobot(Rotation2d.fromRadians(turretToRobotFrameRad(turretTargetRad)));
  } // End stepPositionRad

  /** Get the current hub angle. */
  public Rotation2d getHubAngleRelativeToRobot() {
    return hubAngleRelativeToRobot;
  } // End getHubAngleRelativeToRobot

  /** Get the current Turret position (robot frame: 0 = forward). */
  public Rotation2d getPosition() {
    return Rotation2d.fromRadians(turretInputs.positionRads);
  } // End getPosition

  /** Get the current target position in Turret frame. */
  public Rotation2d getTargetPosition() {
    return Rotation2d.fromRadians(turretInputs.targetPositionRads);
  } // End getTargetPosition

  /** Get the current Turret position in robot frame. 0 = forward, 180° = back. */
  public Rotation2d getRobotFramePosition() {
    return Rotation2d.fromRadians(turretToRobotFrameRad(turretInputs.positionRads));
  } // End getRobotFramePosition

  /** Get the current target position in robot frame. */
  public Rotation2d getRobotFrameTargetPosition() {
    return Rotation2d.fromRadians(turretToRobotFrameRad(turretInputs.targetPositionRads));
  } // End getRobotFrameTargetPosition

  /** Whether the requested robot-frame angle is reachable by the Turret. */
  public boolean isHubInRange() {
    double turretSetpointRad = robotToTurretFrameRad(hubAngleRelativeToRobot.getRadians());
    return turretSetpointRad >= kMinAngleRad && turretSetpointRad <= kMaxAngleRad;
  } // End isHubInRange

  /** Whether the Turret is at the hub within tolerance. */
  public boolean aimedAtHub() {
    double targetTurretRad = getClampedTurretSetpointRad();
    return Math.abs(MathUtil.angleModulus(turretInputs.positionRads - targetTurretRad)) <= kAtHubToleranceRad;
  } // End aimedAtHub

  /** Convert robot-frame angle (0 = forward) to Turret frame (0 = back). */
  private static double robotToTurretFrameRad(double robotFrameRad) {
    return MathUtil.inputModulus(robotFrameRad - Math.PI, -Math.PI, Math.PI);
  } // End robotToTurretFrameRad

  /** Convert Turret-frame angle (0 = back) to robot frame (0 = forward). */
  private static double turretToRobotFrameRad(double turretFrameRad) {
    return MathUtil.inputModulus(turretFrameRad + Math.PI, -Math.PI, Math.PI);
  } // End turretToRobotFrameRad

  /** Hub angle in robot frame clamped to Turret limits after conversion into Turret frame. */
  private double getClampedTurretSetpointRad() {
    return MathUtil.clamp(robotToTurretFrameRad(hubAngleRelativeToRobot.getRadians()), kMinAngleRad, kMaxAngleRad);
  } // End getClampedTurretSetpointRad
}
