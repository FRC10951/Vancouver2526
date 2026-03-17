package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.IoConstants.*;
import frc.robot.Constants.IoConstants.IoCanIdGroup;

/**
 * Fuel system: IO motor, intake, and loader (CAN IDs from
 * Constants.IoConstants), all
 * brushless SPARK MAX.
 */
public class IoSubsystem extends SubsystemBase {
  private final IoCanIdGroup canIds;
  private final SparkMax ioMotor;
  private final RelativeEncoder ioEncoder;
  /** When true, idle state or toggle modes keep shooter spinning. */
  private boolean shooterEnabled = false;
  /** Target shooter speed in RPM for encoder-based control. */
  private double shooterTargetSpeedRpm = 0.0;
  /** When true, idle state runs IO at 50% equivalent speed (survives intake/launch). */
  private boolean spinUp50Requested = false;
  private final SparkMax intakeMotor;
  private final SparkMax loaderMotor;

  public IoSubsystem() {
    this(IO_CAN_IDS);
  }

  public IoSubsystem(IoCanIdGroup canIds) {
    this.canIds = canIds;

    ioMotor = new SparkMax(canIds.ioMotorId, MotorType.kBrushless);
    SparkMaxConfig ioConfig = new SparkMaxConfig();
    ioConfig.smartCurrentLimit(IO_MOTOR_CURRENT_LIMIT);
    ioConfig.voltageCompensation(12.0);
    ioMotor.configure(ioConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    ioEncoder = ioMotor.getEncoder();

    intakeMotor = new SparkMax(canIds.intakeMotorId, MotorType.kBrushless);
    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    intakeConfig.smartCurrentLimit(INTAKE_MOTOR_CURRENT_LIMIT);
    intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    loaderMotor = new SparkMax(canIds.loaderMotorId, MotorType.kBrushless);
    SparkMaxConfig loaderConfig = new SparkMaxConfig();
    loaderConfig.smartCurrentLimit(LOADER_MOTOR_CURRENT_LIMIT);
    loaderMotor.configure(loaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** Set IO motor by voltage and loader by duty cycle (0–1). */
  public void setSpeeds(double ioVoltage, double intakeOutput, double loaderOutput) {
    ioMotor.setVoltage(ioVoltage);
    intakeMotor.setVoltage(intakeOutput);
    loaderMotor.set(loaderOutput);
  }

  /** Returns the current shooter (IO motor) speed in RPM. */
  public double getShooterSpeedRpm() {
    return ioEncoder.getVelocity();
  }

  /** Enables the shooter at the given target speed (RPM). */
  public void enableShooterAtSpeed(double targetSpeedRpm) {
    shooterEnabled = true;
    shooterTargetSpeedRpm = Math.max(0.0, targetSpeedRpm);
  }

  /** Disables the shooter (used when no mode requests it). */
  public void disableShooter() {
    shooterEnabled = false;
    shooterTargetSpeedRpm = 0.0;
    ioMotor.setVoltage(0.0);
  }

  public void stop() {
    setSpeeds(0.0, 0.0, 0.0);
    shooterEnabled = false;
    shooterTargetSpeedRpm = 0.0;
  }

  public Command commandStop() {
    return runOnce(this::stop);
  }

  /** Toggle the "spin 50% when idle" state (X button). Survives intake/launch. */
  public void toggleSpinUp50Requested() {
    spinUp50Requested = !spinUp50Requested;
  }

  /** Toggle shooter enabled state for right-trigger behavior. */
  public void toggleShooterEnabled() {
    shooterEnabled = !shooterEnabled;
    if (shooterEnabled && shooterTargetSpeedRpm <= 0.0) {
      shooterTargetSpeedRpm = SHOOTER_TARGET_SPEED_TOGGLE_RPM;
    }
    if (!shooterEnabled) {
      shooterTargetSpeedRpm = 0.0;
      ioMotor.setVoltage(0.0);
    }
  }

  /** Default command: when no other command runs, apply spin-50 or stop. */
  public Command commandIdle() {
    return run(this::applyIdleState);
  }

  private void applyIdleState() {
    // Determine desired shooter state for idle mode.
    if (spinUp50Requested && !shooterEnabled) {
      // If spin-up-50 is requested and shooter is not already enabled by another mode,
      // enable it at the spin-up-50 target speed.
      enableShooterAtSpeed(SHOOTER_TARGET_SPEED_SPINUP50_RPM);
    } else if (!spinUp50Requested && !shooterEnabled) {
      // No one is requesting shooter; ensure it is fully stopped.
      disableShooter();
    }
    // Intake and loader are idle in this state.
    updateShooterControl();
    intakeMotor.setVoltage(0.0);
    loaderMotor.set(0.0);
  }

  /**
   * Simple closed-loop control for shooter speed using encoder feedback.
   * Uses a spinup region with max voltage, then P-control around a base voltage.
   */
  private void updateShooterControl() {
    if (!shooterEnabled || shooterTargetSpeedRpm <= 0.0) {
      ioMotor.setVoltage(0.0);
      return;
    }

    double currentSpeedRpm = getShooterSpeedRpm();
    double target = shooterTargetSpeedRpm;

    // Spin-up region: below threshold, use max voltage.
    if (currentSpeedRpm < target * SHOOTER_SPINUP_THRESHOLD_FRACTION) {
      double commanded = Math.copySign(SHOOTER_MAX_VOLTAGE, target);
      ioMotor.setVoltage(commanded);
      return;
    }

    // Hold region: P-control around a base voltage.
    double error = target - currentSpeedRpm;
    double commanded = SHOOTER_HOLD_BASE_VOLTAGE + SHOOTER_KP * error;

    // Clamp to max voltage.
    if (commanded > SHOOTER_MAX_VOLTAGE) {
      commanded = SHOOTER_MAX_VOLTAGE;
    } else if (commanded < -SHOOTER_MAX_VOLTAGE) {
      commanded = -SHOOTER_MAX_VOLTAGE;
    }

    ioMotor.setVoltage(commanded);
  }

  public Command commandSpeeds(double ioVoltage, double intakeOutput, double loaderOutput) {
    return startEnd(
        () -> {
          // For legacy callers that still pass an IO voltage, treat that as enabling
          // shooter at the intake target speed; IO voltage itself is now controlled
          // by the encoder loop.
          if (ioVoltage != 0.0) {
            enableShooterAtSpeed(SHOOTER_TARGET_SPEED_INTAKE_RPM);
          }
          intakeMotor.setVoltage(intakeOutput);
          loaderMotor.set(loaderOutput);
        },
        this::stop);
  }

  /**
   * Intake from floor/storage into the robot. IO spins up first, then
   * intake/loader.
   */
  public Command commandIntake() {
    return Commands.sequence(
        // Spin up shooter first using encoder control.
        this.run(() -> enableShooterAtSpeed(SHOOTER_TARGET_SPEED_INTAKE_RPM))
            .withTimeout(INTAKE_SPIN_UP_SECONDS),
        // Then run shooter at speed while feeding intake and loader.
        this.run(
            () -> {
              enableShooterAtSpeed(SHOOTER_TARGET_SPEED_INTAKE_RPM);
              intakeMotor.setVoltage(INTAKING_INTAKE_OUTPUT);
              // Only feed balls once shooter is near target speed.
              double current = getShooterSpeedRpm();
              boolean atSpeed = current >= SHOOTER_TARGET_SPEED_INTAKE_RPM * SHOOTER_SPINUP_THRESHOLD_FRACTION;
              loaderMotor.set(atSpeed ? INTAKING_LOADER_OUTPUT : 0.0);
            })
            .finallyDo(interrupted -> {
              // Stop intake/loader when command ends; shooter may remain enabled if
              // another mode wants it.
              intakeMotor.setVoltage(0.0);
              loaderMotor.set(0.0);
            }));
  }

  public Command commandIntakeAuton() {
    return Commands.sequence(
        this.run(() -> enableShooterAtSpeed(SHOOTER_TARGET_SPEED_INTAKE_RPM))
            .withTimeout(INTAKE_AUTON_SPIN_UP_SECONDS),
        this.run(
            () -> {
              enableShooterAtSpeed(SHOOTER_TARGET_SPEED_INTAKE_RPM);
              intakeMotor.setVoltage(INTAKING_INTAKE_OUTPUT);
              double current = getShooterSpeedRpm();
              boolean atSpeed = current >= SHOOTER_TARGET_SPEED_INTAKE_RPM * SHOOTER_SPINUP_THRESHOLD_FRACTION;
              loaderMotor.set(atSpeed ? INTAKING_LOADER_OUTPUT : 0.0);
            })
            .finallyDo(interrupted -> {
              intakeMotor.setVoltage(0.0);
              loaderMotor.set(0.0);
            }));
  }

  /** Spin up / prepare without fully launching (optional helper). */
  public Command commandPrepare() {
    return this.run(
        () -> {
          enableShooterAtSpeed(SHOOTER_TARGET_SPEED_INTAKE_RPM);
          intakeMotor.setVoltage(INTAKING_INTAKE_OUTPUT);
          double current = getShooterSpeedRpm();
          boolean atSpeed = current >= SHOOTER_TARGET_SPEED_INTAKE_RPM * SHOOTER_SPINUP_THRESHOLD_FRACTION;
          loaderMotor.set(atSpeed ? PREPARING_LOADER_OUTPUT : 0.0);
        }).finallyDo(interrupted -> {
          intakeMotor.setVoltage(0.0);
          loaderMotor.set(0.0);
        });
  }

  /** Launch fuel toward the target (no spin-up delay). */
  public Command commandLaunch() {
    return this.run(
        () -> {
          // Use the same shooter target speed for launching; adjust later if needed.
          enableShooterAtSpeed(SHOOTER_TARGET_SPEED_INTAKE_RPM);
          intakeMotor.setVoltage(INTAKING_INTAKE_OUTPUT);
          double current = getShooterSpeedRpm();
          boolean atSpeed = current >= SHOOTER_TARGET_SPEED_INTAKE_RPM * SHOOTER_SPINUP_THRESHOLD_FRACTION;
          loaderMotor.set(atSpeed ? LAUNCHING_LOADER_OUTPUT : 0.0);
        }).finallyDo(interrupted -> {
          intakeMotor.setVoltage(0.0);
          loaderMotor.set(0.0);
        });
  }

  /** Eject: IO off; intake and loader run in reverse. */
  public Command commandEject() {
    return commandSpeeds(0, -INTAKING_INTAKE_OUTPUT, INTAKING_LOADER_OUTPUT);
  }

  /** IO motor only at 50% (for X button toggle). */
  public Command commandIoSpinUp50() {
    return this.run(() -> enableShooterAtSpeed(SHOOTER_TARGET_SPEED_SPINUP50_RPM));
  }

  public Command commandMaxSpin() {
    return this.run(() -> enableShooterAtSpeed(SHOOTER_TARGET_SPEED_INTAKE_RPM * 1.2));
  }

  /** Reverse flywheel and run loader (Y button). */
  public Command commandReverseFlywheelAndLoader() {
    // Reverse shooter uses open-loop for simplicity; does not use the speed
    // controller because it is only for clearing jams.
    return this.run(
        () -> {
          ioMotor.setVoltage(-SHOOTER_MAX_VOLTAGE / 2.0);
          intakeMotor.setVoltage(0.0);
          loaderMotor.set(INTAKING_LOADER_OUTPUT);
        }).finallyDo(interrupted -> {
          ioMotor.setVoltage(0.0);
          loaderMotor.set(0.0);
        });
  }
}
