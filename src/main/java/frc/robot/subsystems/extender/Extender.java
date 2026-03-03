package frc.robot.subsystems.extender;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.subsystems.extender.ExtenderConstants.kAtTargetRadsTolerance;
import static frc.robot.subsystems.extender.ExtenderConstants.kD;
import static frc.robot.subsystems.extender.ExtenderConstants.kDownExtenderRads;
import static frc.robot.subsystems.extender.ExtenderConstants.kI;
import static frc.robot.subsystems.extender.ExtenderConstants.kP;
import static frc.robot.subsystems.extender.ExtenderConstants.kUpExtenderRads;

public class Extender extends SubsystemBase {

  /** extender Modes:
   * IDLE = stop motor
   * UP = set to target position (Usually UP position)
   * DOWN = set to target position (Usually DOWN position)
   */
  public enum Mode {
    IDLE,
    UP,
    DOWN
  }

  private final ExtenderIO extenderIO;
  private final ExtenderIO.ExtenderIOInputs extenderInputs = new ExtenderIO.ExtenderIOInputs();
  
  private Mode state = Mode.IDLE;

  public Extender(ExtenderIO io) {
    extenderIO = io; 

    SmartDashboard.putNumber("Flywheel/kP", kP);
    SmartDashboard.putNumber("Flywheel/kI", kI);
    SmartDashboard.putNumber("Flywheel/kD", kD);
  }

  @Override
  public void periodic() {
    extenderIO.updateInputs(extenderInputs);
    Logger.recordOutput("Subsystems/extender/Inputs/MotorConnected", extenderInputs.motorConnected);
    Logger.recordOutput("Subsystems/extender/Inputs/AppliedVolts", extenderInputs.appliedVolts);
    Logger.recordOutput("Subsystems/extender/Inputs/SupplyCurrentAmps", extenderInputs.supplyCurrentAmps);
    Logger.recordOutput("Subsystems/extender/Inputs/TargetPositionRads", extenderInputs.targetPositionRads);
    Logger.recordOutput("Subsystems/extender/Inputs/PositionRads", extenderInputs.positionRads);
    Logger.recordOutput("Subsystems/extender/Inputs/VelocityRadsPerSec", extenderInputs.velocityRadsPerSec);

    if (DriverStation.isDisabled()) {
      extenderIO.stop();
      return;
    }

    // Set extender position based on current state
    switch (state) {
      case IDLE:
        extenderIO.stop();
        break;
      case UP:
      case DOWN:
        setTargetRads(extenderInputs.targetPositionRads);
        break;
      default:
        extenderIO.stop();
        break;
    }
  } // End periodic

  /** Set state to idle (Stay at position) */
  public void setIdleMode() {
    state = Mode.IDLE;
  } // End setIdleState

  /** Set state to up (Go to up position) */
  public void setUpMode() {
    state = Mode.UP;
    setTargetRads(kUpExtenderRads);
  } // End setUpState

  /** Set state to down (Go to down position) */
  public void setDownMode() {
    state = Mode.DOWN;
    setTargetRads(kDownExtenderRads);
  } // End setDownState

  /** Sets the motor encoder position to 0 */
  public void resetEncoders() {
    extenderIO.stop();
    extenderIO.resetEncoders();
    setTargetRads(0);
  } // End resetEncoders

  /** Set the target rads, used in UP/DOWN mode */
  public void setTargetRads(double rads) {
    extenderInputs.targetPositionRads = rads;
  } // End setTargetPosition

  /** Returns the target rads */
  public double getTargetRads() {
    return extenderInputs.targetPositionRads;
  } // End getTargetPosition

  /** Get the motors current rads */
  public double getRads() {
    return extenderInputs.positionRads;
  } // End getPosition

  /** Increases the target rads by "steps" */
  public void stepPosition(double steps) {
    setTargetRads(extenderInputs.targetPositionRads + steps);
  } // End stepPosition

  /** Whether the extender is at the target position within tolerance */
  public boolean atTargetPosition() {
    return Math.abs(getRads() - getTargetRads()) <= kAtTargetRadsTolerance;
  } // End atTargetPosition
}
