// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;

/**
 * Drives the robot forward (or backward) until the average encoder distance
 * reaches the target. Use for autonomous routines that need distance-based
 * movement instead of time-based.
 */
public class AutoDriveDistance extends Command {

  private final CANDriveSubsystem driveSubsystem;
  private final double targetDistanceMeters;
  private final double speed;

  /**
   * @param driveSubsystem   The drive subsystem (encoders will be used).
   * @param targetDistanceMeters Target distance in meters; sign matches direction
   *                            (positive = forward, negative = backward).
   * @param speed            Arcade forward speed in [-1, 1]; use negative for
   *                            backward when target is negative.
   */
  public AutoDriveDistance(
      CANDriveSubsystem driveSubsystem,
      double targetDistanceMeters,
      double speed) {
    addRequirements(driveSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.targetDistanceMeters = targetDistanceMeters;
    this.speed = speed;
  }

  @Override
  public void initialize() {
    driveSubsystem.resetEncoders();
  }

  @Override
  public void execute() {
    driveSubsystem.driveArcade(speed, 0);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    double current = driveSubsystem.getAverageDistanceMeters();
    if (targetDistanceMeters >= 0) {
      return current >= targetDistanceMeters;
    }
    return current <= targetDistanceMeters;
  }
}
