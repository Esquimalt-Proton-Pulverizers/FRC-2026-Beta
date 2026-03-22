package frc.robot.subsystems.shooter.hood;

import static frc.robot.subsystems.shooter.hood.HoodConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Hood subsystem: position-controlled mechanism that sets Shooter angle. */
public class Hood extends SubsystemBase {

  private final HoodIO hoodIO;
  private final HoodIO.HoodIOInputs hoodInputs = new HoodIO.HoodIOInputs();

  private double targetAngleRad = kDisabledAngleRad;

  public Hood(HoodIO io) {
    hoodIO = io;
  } // End Hood Constructor

  @Override
  public void periodic() {
    hoodIO.updateInputs(hoodInputs);
    Logger.recordOutput("Subsystems/Shooter/Hood/Inputs/MotorConnected", hoodInputs.motorConnected);
    Logger.recordOutput("Subsystems/Shooter/Hood/TargetDeg", Units.radiansToDegrees(targetAngleRad));
    Logger.recordOutput("Subsystems/Shooter/Hood/PositionDeg", Units.radiansToDegrees(getAngleRad()));
    Logger.recordOutput("Subsystems/Shooter/Hood/Inputs/PositionDeg", Units.radiansToDegrees(hoodInputs.positionRads));
    Logger.recordOutput("Subsystems/Shooter/Hood/Inputs/VelocityDegPerSec", Units.radiansToDegrees(hoodInputs.velocityRadsPerSec));
    Logger.recordOutput("Subsystems/Shooter/Hood/Inputs/AppliedVolts", hoodInputs.appliedVolts);
    Logger.recordOutput("Subsystems/Shooter/Hood/Inputs/SupplyCurrentAmps", hoodInputs.supplyCurrentAmps);

    if (DriverStation.isDisabled()) {
      hoodIO.stop();
      return;
    }

    // Set the Hood target position from clamped angle
    double clampedRad = MathUtil.clamp(targetAngleRad, kMinAngleRad, kMaxAngleRad);
    double targetPositionRad = clampedRad + kEncoderZeroOffsetRad;
    hoodIO.setTargetPosition(targetPositionRad);
  } // End periodic

  /** Set the target angle. Clamped to min/max in periodic. */
  public void setTargetAngleRad(double angleRad) {
    targetAngleRad = angleRad;
  } // End setTargetAngleRad

  /** Get the current target angle. */
  public double getTargetAngleRad() {
    return targetAngleRad;
  } // End getTargetAngleRad

  /** Get the current Hood angle. */
  public double getAngleRad() {
    return hoodInputs.positionRads + kEncoderZeroOffsetRad;
  } // End getAngleRad

  /** Whether the Hood is at the target angle within tolerance. */
  public boolean atTarget() {
    double currentRad = hoodInputs.positionRads + kEncoderZeroOffsetRad;
    double targetClamped = MathUtil.clamp(targetAngleRad, kMinAngleRad, kMaxAngleRad);
    return Math.abs(currentRad - targetClamped) <= kAtTargetToleranceRad;
  } // End atTarget
}
