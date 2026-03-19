// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import static frc.robot.Constants.OperatorConstants.*;

import frc.robot.commands.AutoDrive;
import frc.robot.commands.AutoDriveDistance;
import frc.robot.commands.AutoTurn;
import frc.robot.commands.Drive;
import static frc.robot.Constants.DriveConstants.AUTO_DRIVE_SPEED;
import static frc.robot.Constants.DriveConstants.AUTO_TURN_SPEED;

import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.IoSubsystem;

//import frc.robot.subsystems.IoSubsystem.commandLaunch;

/**
 * Robot container: subsystems, driver/operator controllers, button bindings,
 * and autonomous chooser. See README.md for control layout and CAN IDs.
 */
public class RobotContainer {
  private final CANDriveSubsystem driveSubsystem = new CANDriveSubsystem();
  private final IoSubsystem ioSubsystem = new IoSubsystem();
  private final CommandXboxController driverController = new CommandXboxController(DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController = new CommandXboxController(OPERATOR_CONTROLLER_PORT);
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();

    autoChooser.setDefaultOption("Match Auto", Commands.sequence(
        // 1. Forward 0.5 m
        new AutoDriveDistance(driveSubsystem, 0.5, AUTO_DRIVE_SPEED).withTimeout(4.0),
        // 2. Turn 90° right
        new AutoTurn(driveSubsystem, 90.0, Math.min(1.0, AUTO_TURN_SPEED * 2.5)).withTimeout(3.0),
        // 3. Forward 1.9 m
        new AutoDriveDistance(driveSubsystem, 1.9, AUTO_DRIVE_SPEED).withTimeout(8.0),
        // 4. Turn 90° left
        new AutoTurn(driveSubsystem, -90.0, Math.min(1.0, AUTO_TURN_SPEED * 2.5)).withTimeout(5.0),
        // 5–7. Intake runs for 10 s in parallel with the remaining drive steps
        Commands.parallel(
            ioSubsystem.commandIntake().withTimeout(10.0),
            Commands.sequence(
                // 6. Forward 2.8 m
                new AutoDriveDistance(driveSubsystem, 2.8, AUTO_DRIVE_SPEED).withTimeout(10.0),
                // 7. Turn 30° right
                new AutoTurn(driveSubsystem, 30.0, Math.min(1.0, AUTO_TURN_SPEED * 2.5)).withTimeout(4.0)))));

    autoChooser.addOption("Do Nothing", Commands.none());
    autoChooser.addOption("Drive Forward 2s", new AutoDrive(driveSubsystem, 0.5, 0.0).withTimeout(2.0));

    autoChooser.addOption("(test) turn left 2s",
        new AutoDrive(driveSubsystem, 0, 0.1).withTimeout(2.0)
            .andThen(Commands.none()));

    autoChooser.addOption("(untested) Drive back & shoot preload left",
        new AutoDrive(driveSubsystem, -0.5, 0.0).withTimeout(2.0)
            .andThen(new AutoDrive(driveSubsystem, 0, 0.1).withTimeout(2.0))
            .andThen(ioSubsystem.commandLaunch().withTimeout(1.0))
            .andThen(Commands.none()));

    autoChooser.addOption("(untested) Drive back & shoot preload right",
        new AutoDrive(driveSubsystem, -0.5, 0.0).withTimeout(2.0)
            .andThen(new AutoDrive(driveSubsystem, 0, -0.1).withTimeout(2.0))
            .andThen(ioSubsystem.commandLaunch().withTimeout(1.0))
            .andThen(Commands.none()));

    SmartDashboard.putData("Auto", autoChooser);
  }

  public CANDriveSubsystem getDriveSubsystem() {
    return driveSubsystem;
  }

  public IoSubsystem getIoSubsystem() {
    return ioSubsystem;
  }

  public CommandXboxController getOperatorController() {
    return operatorController;
  }

  private void configureBindings() {
    driveSubsystem.setDefaultCommand(new Drive(driveSubsystem, driverController));
    ioSubsystem.setDefaultCommand(ioSubsystem.commandIdle());

    // Left trigger SHOOTS: spin up shooter with encoder control and feed balls
    // (loader gated on shooter speed) while allowing normal driving.
    driverController.leftTrigger(TRIGGER_THRESHOLD).whileTrue(ioSubsystem.commandLaunch());

    // Right trigger INTAKES: run intake + loader only (no shooter spin).
    driverController.rightTrigger(TRIGGER_THRESHOLD).whileTrue(ioSubsystem.commandIntake());
    driverController.leftBumper().whileTrue(ioSubsystem.commandEject());
    driverController.x().onTrue(Commands.runOnce(ioSubsystem::toggleSpinUp50Requested));
    driverController.y().whileTrue(ioSubsystem.commandReverseFlywheelAndLoader());
    driverController.b().onTrue(driveSubsystem.runOnce(driveSubsystem::resetEncoders));
    driverController.a().whileTrue(ioSubsystem.commandHighSpeedLaunch());
  }

  /**
   * Autonomous: start at center, drive toward our HUB to shooting range, then
   * launch into HUB.
   * Assumes robot is facing the alliance HUB. Tune CENTER_TO_SHOOT_DRIVE_METERS
   * for your shooter.
   */
  /** Returns the autonomous command selected on the dashboard. */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
