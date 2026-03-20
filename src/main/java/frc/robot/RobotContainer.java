// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import static frc.robot.Constants.OperatorConstants.*;
import static frc.robot.Constants.DriveConstants.*;

import frc.robot.commands.AutoDrive;
import frc.robot.commands.Drive;

import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.IoSubsystem;

import java.util.Optional;
import java.util.OptionalInt;

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

    autoChooser.setDefaultOption("Timed auto", autonomousCommand());
    autoChooser.addOption("Do Nothing", Commands.none());
    autoChooser.addOption("Drive Forward 2s", new AutoDrive(driveSubsystem, 0.5, 0.0).withTimeout(2.0));
    SmartDashboard.putData("Auto choices", autoChooser);
  }

  private Command autonomousRedStation1() {
    return Commands.none();
  }

  private Command autonomousRedStation2() {
    return Commands.none();
  }

  private Command autonomousRedStation3() {
    return Commands.none();
  }

  private Command autonomousBlueStation1() {
    return autonomousCommand();
  }

  private Command autonomousBlueStation2() {
    return Commands.none();
  }

  private Command autonomousBlueStation3() {
    return Commands.none();
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

  /**
   * Wrapper used for teleop shooting: launch + drivetrain wiggle.
   * Intake/feed remains continuous from the launch command (no pulsing).
   */
  private Command createShootingSequence(Command launchCommand) {
    return Commands.parallel(
        launchCommand,
        driveSubsystem.commandIntakeWiggle(INTAKE_WIGGLE_SPEED, 0.10));
  }

  private void configureBindings() {
    driveSubsystem.setDefaultCommand(new Drive(driveSubsystem, driverController));
    ioSubsystem.setDefaultCommand(ioSubsystem.commandIdle());

    // Left trigger SHOOTS: spin up shooter with encoder control and feed balls
    // (loader gated on shooter speed) while allowing normal driving.
    driverController.leftTrigger(TRIGGER_THRESHOLD).whileTrue(
        createShootingSequence(ioSubsystem.commandLaunch()));

    // Right trigger INTAKES: run intake + loader only (no shooter spin).
    driverController.rightTrigger(TRIGGER_THRESHOLD).whileTrue(ioSubsystem.commandIntake());
    driverController.leftBumper().whileTrue(ioSubsystem.commandEject());
    driverController.x().onTrue(Commands.runOnce(ioSubsystem::toggleSpinUp50Requested));
    driverController.y().whileTrue(ioSubsystem.commandReverseFlywheelAndLoader());
    // Range buttons (recovered from pre-merge teleop):
    // A = normal/high shot, B = faster shot, RB = long-range shot.
    driverController.a().whileTrue(createShootingSequence(ioSubsystem.commandLaunch()));
    driverController.b().whileTrue(createShootingSequence(ioSubsystem.commandHighSpeedLaunch()));
    driverController.rightBumper().whileTrue(createShootingSequence(ioSubsystem.commandUltraSpeedLaunch()));
    // Keep encoder reset available on start.
    driverController.start().onTrue(driveSubsystem.runOnce(driveSubsystem::resetEncoders));
  }

  /**
   * Shoot then timed {@link AutoDrive} segments — wired to
   * {@link #autonomousBlueStation1()}.
   */

  // ------------------------------------------------------------
  // ------------------------------------------------------------
  // MAIN AUTONOMOUS ROUTINE
  // ------------------------------------------------------------
  // ------------------------------------------------------------
  public Command autonomousCommand() {
    return Commands.sequence(
        // Shoot initial 8 fuel;
        ioSubsystem.commandLaunch().withTimeout(5.0),

        // Drive forward 0.5 meters;
        new AutoDrive(driveSubsystem, 0.75, 0.0).withTimeout(0.5),

        // Rotate 90 degrees to the right;
        new AutoDrive(driveSubsystem, 0.0, -0.66).withTimeout(0.5),

        // Drive forward 0.5 meters;
        new AutoDrive(driveSubsystem, 0.75, 0.0).withTimeout(1.0),

        // Rotate 90 degrees to the right;
        new AutoDrive(driveSubsystem, 0.0, 0.64).withTimeout(0.5),

        // Intake while driving forward 2.8 meters (no intake timeout; drive is the
        // deadline).
        new ParallelDeadlineGroup(
            new AutoDrive(driveSubsystem, 0.75, 0.0).withTimeout(0.87),
            ioSubsystem.commandIntake()),

        // Rotate 30 degrees to the right;
        new AutoDrive(driveSubsystem, 0.0, 0.5).withTimeout(0.5),

        // Shoot for 5 seconds;
        ioSubsystem.commandLaunch().withTimeout(5.0));
  }

  // ------------------------------------------------------------
  // ------------------------------------------------------------

  /**
   * Red/Blue stations 1–3 each map to a dedicated routine (see
   * {@code autonomous*Station*}). Five
   * slots are {@link Commands#none()} until you fill them; Blue 1 runs
   * {@link #autonomousCommand()}.
   * If alliance or station is unknown, uses SmartDashboard {@code Auto choices}.
   */
  public Command getAutonomousCommand() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    OptionalInt station = DriverStation.getLocation();
    if (alliance.isPresent() && station.isPresent()) {
      int location = station.getAsInt();
      if (location >= 1 && location <= 3) {
        if (alliance.get() == Alliance.Red) {
          return switch (location) {
            case 1 -> autonomousRedStation1();
            case 2 -> autonomousRedStation2();
            case 3 -> autonomousRedStation3();
            default -> Commands.none();
          };
        }
        if (alliance.get() == Alliance.Blue) {
          return switch (location) {
            case 1 -> autonomousBlueStation1();
            case 2 -> autonomousBlueStation2();
            case 3 -> autonomousBlueStation3();
            default -> Commands.none();
          };
        }
      }
    }
    Command selected = autoChooser.getSelected();
    return selected != null ? selected : Commands.none();
  }
}
