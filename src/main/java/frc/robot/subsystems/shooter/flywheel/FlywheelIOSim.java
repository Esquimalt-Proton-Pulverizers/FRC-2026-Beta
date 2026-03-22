package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;

/** Flywheel IO for simulation; slew-rate-limited setpoint following. */
public class FlywheelIOSim implements FlywheelIO {

  /** Same ramp rate as hardware: 1000 RPM/s → 2 s to 2000 RPM, 4 s to 4000 RPM (rad/s²). */
  private static final double kVelocityRampRateRadPerSecSq =
      Units.rotationsPerMinuteToRadiansPerSecond(FlywheelConstants.kVelocityRampRateRpmPerSec);

  private final SlewRateLimiter slewRateLimiter = new SlewRateLimiter(kVelocityRampRateRadPerSecSq);

  private double targetVelocityRadPerSec = 0.0;
  private boolean isStopped = false;

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    if (isStopped) {
      targetVelocityRadPerSec = 0.0;
    }
    double currentVelocityRadPerSec = slewRateLimiter.calculate(targetVelocityRadPerSec);

    inputs.motorConnected = true;
    inputs.velocityRadsPerSec = currentVelocityRadPerSec;
    inputs.appliedVolts = 0.0;
    inputs.supplyCurrentAmps = 0.0;
  } // End updateInputs

  @Override
  public void setTargetVelocity(double targetVelocityRadPerSec) {
    this.targetVelocityRadPerSec = targetVelocityRadPerSec;
    isStopped = false;
  } // End setTargetVelocity

  @Override
  public void stop() {
    isStopped = true;
  } // End stop
}
