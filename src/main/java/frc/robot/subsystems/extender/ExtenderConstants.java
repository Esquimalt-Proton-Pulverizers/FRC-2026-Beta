package frc.robot.subsystems.extender;

import com.revrobotics.spark.config.SparkBaseConfig;

public class ExtenderConstants {

  // TODO: get motor id
  /** CAN ID of the extender */
  public static final double kMotorId = 0;

  // TODO: tune position
  /** Target position when the extender is in the UP mode */
  public static final double kUpExtenderRads = 0;

  // TODO: tune position
  /** Target position when the extender is in the DOWN mode */
  public static final double kDownExtenderRads = 0;

  /** Set true to invert the motor */
  public static final boolean kMotorInverted = false;

  // TODO: tune max pos
  /** Max radians for the extender to rotate */
  public static final double kMaxRads = 0.5;
  
  // TODO: tune min voltage
  /** Min radians for the extender to rotate */
  public static final double kMinRads = 0;

  // TODO: tune PID
  /** PID values for to-position target */
  public static final double kP = 0;
  public static final double kI = 0;
  public static final double kD = 0;

  // TODO: tune
  /** Tolerance for at-target position */
  public static final double kAtTargetRadsTolerance = 0;
}
