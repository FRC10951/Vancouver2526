package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.OperatorConstants.DRIVER_CONTROLLER_PORT;
import static frc.robot.Constants.OperatorConstants.TRIGGER_THRESHOLD;

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
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();

    autoChooser.setDefaultOption("Timed auto (full)", autonomousCommand());
    autoChooser.addOption("Do Nothing", Commands.none().withName("Chooser: Do Nothing"));
    autoChooser.addOption("Shoot only", chooserShootOnly());
    autoChooser.addOption("Drive forward 1.5s", chooserDriveForwardShort());
    autoChooser.addOption("Shoot then forward 1.5s", chooserShootThenForwardShort());
    autoChooser.addOption("Drive backward 1.5s", chooserDriveBackwardShort());
    autoChooser.addOption(
        "Drive forward 2s",
        new AutoDrive(driveSubsystem, CHOOSER_DRIVE_2S_SPEED, 0.0)
            .withTimeout(CHOOSER_DRIVE_2S_SECONDS)
            .withName("Chooser: Drive forward 2s"));
    SmartDashboard.putData("Robot/Auto choices", autoChooser);
  }

  /** Publishes chooser selection for dashboards (command {@link Command#getName()}). */
  public void publishDashboardPeriodic() {
    Command sel = autoChooser.getSelected();
    SmartDashboard.putString("Robot/Auto selected", sel != null ? sel.getName() : "(none)");
  }

  private Command chooserShootOnly() {
    return ioSubsystem
        .commandLaunch()
        .withTimeout(CHOOSER_SHOOT_ONLY_SECONDS)
        .withName("Chooser: Shoot only");
  }

  private Command chooserDriveForwardShort() {
    return new AutoDrive(driveSubsystem, CHOOSER_SIMPLE_FWD_SPEED, 0.0)
        .withTimeout(CHOOSER_SIMPLE_FWD_SECONDS)
        .withName("Chooser: Drive forward 1.5s");
  }

  private Command chooserShootThenForwardShort() {
    return Commands.sequence(
            ioSubsystem.commandLaunch().withTimeout(CHOOSER_SHOOT_THEN_FWD_SHOOT_SECONDS),
            new AutoDrive(driveSubsystem, CHOOSER_SIMPLE_FWD_SPEED, 0.0)
                .withTimeout(CHOOSER_SIMPLE_FWD_SECONDS))
        .withName("Chooser: Shoot then forward 1.5s");
  }

  private Command chooserDriveBackwardShort() {
    return new AutoDrive(driveSubsystem, CHOOSER_SIMPLE_REV_SPEED, 0.0)
        .withTimeout(CHOOSER_SIMPLE_REV_SECONDS)
        .withName("Chooser: Drive backward 1.5s");
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
    driverController.a().whileTrue(createShootingSequence(ioSubsystem.commandHighSpeedLaunch()));
    driverController.b().whileTrue(createShootingSequence(ioSubsystem.commandUltraSpeedLaunch()));
    driverController.rightBumper().whileTrue(createShootingSequence(ioSubsystem.commandUltraSpeedLaunch()));
    if (DRIVE_QUADRATURE_ENCODERS_WIRED) {
      driverController.start().onTrue(driveSubsystem.runOnce(driveSubsystem::resetEncoders));
    }
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
        ioSubsystem.commandLaunch().withTimeout(AUTO_INITIAL_SHOOT_SECONDS),

        new AutoDrive(driveSubsystem, AUTO_FWD1_SPEED, 0.0).withTimeout(AUTO_FWD1_SECONDS),

        new AutoDrive(driveSubsystem, 0.0, AUTO_TURN1_ROTATION).withTimeout(AUTO_TURN1_SECONDS),

        new AutoDrive(driveSubsystem, AUTO_FWD2_SPEED, 0.0).withTimeout(AUTO_FWD2_SECONDS),

        new AutoDrive(driveSubsystem, 0.0, AUTO_TURN2_ROTATION).withTimeout(AUTO_TURN2_SECONDS),

        new ParallelDeadlineGroup(
            new AutoDrive(driveSubsystem, AUTO_FWD_INTAKE_SPEED, 0.0).withTimeout(AUTO_FWD_INTAKE_SECONDS),
            ioSubsystem.commandIntake()),

        new AutoDrive(driveSubsystem, 0.0, AUTO_TURN3_ROTATION).withTimeout(AUTO_TURN3_SECONDS),

        ioSubsystem.commandLaunch().withTimeout(AUTO_FINAL_SHOOT_SECONDS))
        .withName("Timed auto (full)");
  }

  // ------------------------------------------------------------
  // ------------------------------------------------------------

  /**
   * Red/Blue stations 1–3 each map to a dedicated routine (see
   * {@code autonomous*Station*}). Five
   * slots are {@link Commands#none()} until you fill them; Blue 1 runs
   * {@link #autonomousCommand()}.
   * If alliance or station is unknown, uses SmartDashboard {@code Robot/Auto choices}.
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
