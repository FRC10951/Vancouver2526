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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.Drive;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.IoSubsystem;
import java.util.Optional;
import java.util.OptionalInt;
import static frc.robot.Constants.IoConstants.SHOOTER_TARGET_SPEED_HIGH_RPM;
import static frc.robot.Constants.IoConstants.SHOOTER_TARGET_SPEED_ULTRA_RPM;
import static frc.robot.Constants.OperatorConstants.*;

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
    autoChooser.addOption(
        "Drive Forward 2s", new AutoDrive(driveSubsystem, 0.5, 0.0).withTimeout(2.0));
    SmartDashboard.putData("Auto choices", autoChooser);
  }

  private Command autonomousRedStation1() {
    return autonomousCommand();
  }

  private Command autonomousRedStation2() {
    return autonomousCommand();
  }

  private Command autonomousRedStation3() {
    return autonomousCommand();
  }

  private Command autonomousBlueStation1() {
    return autonomousCommand();
  }

  private Command autonomousBlueStation2() {
    return autonomousCommand();
  }

  private Command autonomousBlueStation3() {
    return autonomousCommand();
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
        launchCommand, driveSubsystem.commandIntakeWiggleWhileShooting());
  }

  /**
   * Like {@link #createShootingSequence(Command)} but waits for flywheel spin-up
   * (launch phase 1) before starting the intake wiggle alongside sustained shoot.
   */
  private Command createShootingSequenceAfterSpinUp(double shooterRpm) {
    return Commands.sequence(
        ioSubsystem.commandLaunchSpinUpPhase(shooterRpm),
        Commands.parallel(
            ioSubsystem.commandLaunchSustainPhase(shooterRpm),
            driveSubsystem.commandIntakeWiggleWhileShooting()));
  }

  private void configureBindings() {
    driveSubsystem.setDefaultCommand(new Drive(driveSubsystem, driverController));
    ioSubsystem.setDefaultCommand(ioSubsystem.commandIdle());

    // Left trigger SHOOTS: spin up shooter with encoder control and feed balls
    // (loader gated on shooter speed) while allowing normal driving.
    driverController
        .leftTrigger(TRIGGER_THRESHOLD)
        .whileTrue(createShootingSequence(ioSubsystem.commandLaunch()));

    // Right trigger INTAKES: run intake + loader only (no shooter spin).
    driverController.rightTrigger(TRIGGER_THRESHOLD).whileTrue(ioSubsystem.commandIntake());
    driverController.leftBumper().whileTrue(ioSubsystem.commandEject());
    driverController.x().onTrue(Commands.runOnce(ioSubsystem::toggleSpinUp50Requested));
    driverController.y().whileTrue(ioSubsystem.commandReverseFlywheelAndLoader());
    // Range buttons (recovered from pre-merge teleop):
    // A = normal/high shot, B = faster shot, RB = long-range shot.
    driverController
        .a()
        .whileTrue(createShootingSequenceAfterSpinUp(SHOOTER_TARGET_SPEED_HIGH_RPM));
    driverController
        .b()
        .whileTrue(createShootingSequenceAfterSpinUp(SHOOTER_TARGET_SPEED_ULTRA_RPM));
    driverController
        .rightBumper()
        .whileTrue(createShootingSequence(ioSubsystem.commandUltraSpeedLaunch()));
    // Keep encoder reset available on start.
    driverController.start().onTrue(driveSubsystem.runOnce(driveSubsystem::resetEncoders));
  }

  /**
   * Shoot then timed {@link AutoDrive} segments - wired to
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

        ioSubsystem.commandIntake().withTimeout(8),

        // Drive forward 0.5 meters;
        new AutoDrive(driveSubsystem, 0.75, 0.0).withTimeout(0.25));
  }

  // ------------------------------------------------------------
  // ------------------------------------------------------------

  /**
   * Red/Blue stations 1-3 each map to a dedicated routine (see
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