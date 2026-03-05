package frc.robot.subsystems.hang;

import static frc.robot.subsystems.hang.HangConstants.*;

import edu.wpi.first.math.MathUtil;

/**
 * Hang IO for simulation: potentiometer voltage moves toward min/max based on
 * applied motor voltage; no real motor or encoder.
 */
public class HangIOSim implements HangIO {

  private static final double kLoopPeriodSec = 0.02;

  /** Simulated pot rate (V/s at 12 V). Full range (5 - 1.65 V) in ~1.5 s. */
  private static final double kSimPotRateVPerSecAt12V =
      (kPotRetractedVoltage - kPotExtendedVoltage) / 1.5;

  private double simulatedPotVoltage = kPotRetractedVoltage; // Start retracted (5 V)
  private double appliedVolts = 0.0;
  private boolean isStopped = true;

  @Override
  public void updateInputs(HangIOInputs inputs) {
    if (!isStopped) {
      // Positive voltage = extend = pot voltage decreases
      double rate = kSimPotRateVPerSecAt12V * (appliedVolts / kMaxVoltage);
      simulatedPotVoltage -= rate * kLoopPeriodSec;
      simulatedPotVoltage = MathUtil.clamp(simulatedPotVoltage, kPotExtendedVoltage, kPotRetractedVoltage);
    }

    inputs.motorConnected = true;
    inputs.appliedVolts = isStopped ? 0.0 : appliedVolts;
    inputs.supplyCurrentAmps = isStopped ? 0.0 : Math.abs(appliedVolts) / kMaxVoltage * 2.0;
    inputs.potVoltage = simulatedPotVoltage;
  } // End updateInputs

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
    isStopped = false;
  } // End setVoltage

  @Override
  public void stop() {
    isStopped = true;
    appliedVolts = 0.0;
  } // End stop
}
