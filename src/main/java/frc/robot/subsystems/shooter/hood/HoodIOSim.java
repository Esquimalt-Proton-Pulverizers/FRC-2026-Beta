package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
/** Hood IO for simulation; slew-rate-limited setpoint following. */
public class HoodIOSim implements HoodIO {

  private final SlewRateLimiter slewRateLimiter = new SlewRateLimiter(HoodConstants.kSimMaxSlewRadPerSec);

  private double targetPositionRad = 0.0;
  private double limitedPositionRad = HoodConstants.kDisabledAngleRad;
  private boolean isStopped = false;

  public HoodIOSim() {
    slewRateLimiter.reset(HoodConstants.kDisabledAngleRad);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    if (!isStopped) {
      double clampedTarget = MathUtil.clamp(targetPositionRad, HoodConstants.kMinAngleRad, HoodConstants.kMaxAngleRad);
      limitedPositionRad = slewRateLimiter.calculate(clampedTarget);
    }

    inputs.motorConnected = true;
    inputs.positionRads = limitedPositionRad;
    inputs.velocityRadsPerSec = 0.0;
    inputs.appliedVolts = 0.0;
    inputs.supplyCurrentAmps = 0.0;
  } // End updateInputs

  @Override
  public void setTargetPosition(double targetPositionRad) {
    this.targetPositionRad = targetPositionRad;
    isStopped = false;
  } // End setTargetPosition

  @Override
  public void stop() {
    isStopped = true;
  } // End stop
}
