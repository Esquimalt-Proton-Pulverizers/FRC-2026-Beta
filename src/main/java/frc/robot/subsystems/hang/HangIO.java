package frc.robot.subsystems.hang;

import org.littletonrobotics.junction.AutoLog;

/** IO interface for the Hang (climber) subsystem using a SPARK MAX and analog potentiometer. */
public interface HangIO {

  @AutoLog
  class HangIOInputs {
    public boolean motorConnected = false;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;

    /** Raw potentiometer voltage from the analog input. */
    public double potVoltage = 0.0;
  }

  /** Update inputs from the hardware. */
  default void updateInputs(HangIOInputs inputs) {}

  /** Set the motor output voltage for the hang mechanism. */
  default void setVoltage(double volts) {}

  /** Stop the motor (coast or brake, depending on configuration). */
  default void stop() {}
}

