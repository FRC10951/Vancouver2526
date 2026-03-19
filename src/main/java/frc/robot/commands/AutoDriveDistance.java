package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;

/**
 * Drives a specific distance using encoder feedback. Resets encoders on start
 * and monitors average wheel distance until the target is reached.
 *
 * <p>Always pair with {@code .withTimeout()} as a safety net in case the
 * encoders malfunction or the robot is physically blocked.
 */
public class AutoDriveDistance extends Command {

  private final CANDriveSubsystem driveSubsystem;
  private final double targetMeters;
  private final double speed;

  /**
   * @param driveSubsystem The drive subsystem
   * @param targetMeters   Distance to travel (positive = forward, negative = backward)
   * @param speed          Absolute duty-cycle speed [0, 1]
   */
  public AutoDriveDistance(CANDriveSubsystem driveSubsystem, double targetMeters, double speed) {
    addRequirements(driveSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.targetMeters = targetMeters;
    this.speed = speed;
  }

  @Override
  public void initialize() {
    driveSubsystem.resetEncoders();
  }

  @Override
  public void execute() {
    driveSubsystem.driveArcade(Math.signum(targetMeters) * speed, 0);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(driveSubsystem.getAverageDistanceMeters()) >= Math.abs(targetMeters);
  }
}
