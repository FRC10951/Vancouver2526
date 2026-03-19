package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;
import static frc.robot.Constants.DriveConstants.TRACK_WIDTH_METERS;

/**
 * Point-turns the robot by a specified angle using encoder-based differential
 * measurement. Angle is computed as (leftDistance − rightDistance) / trackWidth.
 *
 * <p>Uses {@link edu.wpi.first.wpilibj.drive.DifferentialDrive#arcadeDrive} with
 * zRotation per WPILib convention: counterclockwise positive. So positive
 * angleDegrees = clockwise (right) = negative zRotation.
 *
 * <p>Accuracy depends on the {@code TRACK_WIDTH_METERS} constant matching
 * the real robot. Always pair with {@code .withTimeout()} as a safety net.
 */
public class AutoTurn extends Command {

  private final CANDriveSubsystem driveSubsystem;
  private final double targetAngleRad;
  private final double speed;

  /**
   * @param driveSubsystem The drive subsystem
   * @param angleDegrees   Angle to turn (positive = clockwise/right, negative = CCW/left)
   * @param speed          Absolute rotation duty-cycle [0, 1]
   */
  public AutoTurn(CANDriveSubsystem driveSubsystem, double angleDegrees, double speed) {
    addRequirements(driveSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.targetAngleRad = Math.toRadians(angleDegrees);
    this.speed = speed;
  }

  @Override
  public void initialize() {
    driveSubsystem.resetEncoders();
  }

  @Override
  public void execute() {
    // DifferentialDrive: zRotation positive = counterclockwise. Clockwise (right) = negative.
    driveSubsystem.driveArcade(0, -Math.signum(targetAngleRad) * speed, false); // linear for auton
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    double leftDist = driveSubsystem.getLeftDistanceMeters();
    double rightDist = driveSubsystem.getRightDistanceMeters();
    double currentAngleRad = (leftDist - rightDist) / TRACK_WIDTH_METERS;
    return Math.abs(currentAngleRad) >= Math.abs(targetAngleRad);
  }
}
