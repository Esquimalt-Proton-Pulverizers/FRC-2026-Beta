package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hang.Hang;
import frc.robot.subsystems.hang.PostAlignConstants;
import frc.robot.subsystems.hang.PostDetectionReader;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * While held: pathfind to hang pose with hang at Level 1, align using PostDetection if the stream is
 * live, optional final strafe, then STORED. Rumble on early exit (no vision) or full success. POV
 * release cancels and idles hang.
 */
public class AutoHangCommand extends Command {

  private final Command delegate;
  private final Drive drive;
  private final Hang hang;

  /**
   * @param pathName PathPlanner path under deploy/pathplanner/paths (e.g. HangingPosition-Left).
   */
  public static Command create(
      Drive drive,
      Hang hang,
      CommandXboxController driverController,
      String pathName,
      PostDetectionReader reader) {

    AtomicBoolean alignOk = new AtomicBoolean(false);
    AlignToPostCommand align = new AlignToPostCommand(drive, reader, alignOk);

    Command optionalInch = Commands.either(
            Commands.run(() -> drive.runVelocity(
              new ChassisSpeeds(0.0, PostAlignConstants.kFinalInchVyMetersPerSec, 0.0)),
              drive).withTimeout(PostAlignConstants.kFinalInchSeconds),
            Commands.none(),
            () -> Math.abs(PostAlignConstants.kFinalInchVyMetersPerSec) > 1e-6);

    Command phase1 =
        Commands.deadline(
            DriveCommands.pathfindThenFollowPath(drive, pathName),
            Commands.runOnce(() -> hang.setLevel1State(), hang));

    Command visionBranch =
        Commands.sequence(
            align,
            Commands.either(
                Commands.sequence(
                    optionalInch,
                    Commands.runOnce(() -> hang.setStoredState(), hang),
                    shortRumble(driverController)),
                Commands.sequence(
                    Commands.runOnce(() -> hang.setIdleState(), hang),
                    shortRumble(driverController)),
                alignOk::get));

    Command earlyBranch =
        Commands.sequence(
            Commands.runOnce(() -> hang.setIdleState(), hang), shortRumble(driverController));

    Command branch = Commands.either(visionBranch, earlyBranch, reader::isPublishing);

    Command sequence = Commands.sequence(phase1, branch);
    return new AutoHangCommand(sequence, drive, hang);
  } // End create

  private static Command shortRumble(CommandXboxController controller) {
    return Commands.startEnd(
            () -> controller
                .getHID()
                .setRumble(RumbleType.kBothRumble, Constants.ControllerConstants.CONTROLLER_RUMBLE_STRENGTH),
            () -> controller.getHID().setRumble(RumbleType.kBothRumble, 0.0))
        .withTimeout(Constants.ControllerConstants.CONTROLLER_RUMBLE_PULSE_SECONDS)
        .withName("AutoHangRumble");
  } // End shortRumble

  private AutoHangCommand(Command delegate, Drive drive, Hang hang) {
    this.delegate = delegate;
    this.drive = drive;
    this.hang = hang;
    for (Subsystem subsystem : delegate.getRequirements()) {
      addRequirements(subsystem);
    }
  } // End AutoHangCommand Constructor

  @Override
  public void initialize() {
    delegate.initialize();
  } // End initialize

  @Override
  public void execute() {
    delegate.execute();
  } // End execute

  @Override
  public void end(boolean interrupted) {
    delegate.end(interrupted);
    drive.stop();
    if (interrupted) {
      hang.setIdleState();
    }
  } // End end

  @Override
  public boolean isFinished() {
    return delegate.isFinished();
  } // End isFinished
} // End AutoHangCommand class
