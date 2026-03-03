// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.OperatorConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CANDriveSubsystem;

/**
 * Default split-arcade drive command.
 *
 * <p>Left stick Y-axis: forward/backward. Right stick X-axis: rotation (turn).
 * Pushing the left stick forward drives forward; pushing the right stick right
 * turns the robot right.
 */
public class Drive extends Command {

  private final CANDriveSubsystem driveSubsystem;
  private final CommandXboxController controller;

  public Drive(CANDriveSubsystem driveSystem, CommandXboxController driverController) {
    addRequirements(driveSystem);
    driveSubsystem = driveSystem;
    controller = driverController;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double xSpeed = -controller.getLeftY() * DRIVE_SCALING;
    double zRotation = controller.getRightX() * ROTATION_SCALING;
    driveSubsystem.driveArcade(xSpeed, zRotation);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}