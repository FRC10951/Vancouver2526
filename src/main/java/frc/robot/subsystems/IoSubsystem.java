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
  private final SparkMax ioMotor;
  private final RelativeEncoder ioEncoder;
  /** When true, idle state or toggle modes keep shooter spinning. */
  private boolean shooterEnabled = false;
  /** Target shooter speed in RPM for encoder-based control. */
  private double shooterTargetSpeedRpm = 0.0;
  /**
   * When true, idle state runs IO at 50% equivalent speed (survives
   * intake/launch).
   */
  private boolean spinUp50Requested = false;
  private final SparkMax intakeMotor;
  private final SparkMax loaderMotor;

  public IoSubsystem() {
    this(IO_CAN_IDS);
  }

  public IoSubsystem(IoCanIdGroup canIds) {
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

  @Override
  public void periodic() {
    // Keep shooter speed control running every cycle whenever it is enabled.
    updateShooterControl();
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

  /** Default command: when no other command runs, apply spin-50 or stop. */
  public Command commandIdle() {
    return run(this::applyIdleState);
  }

  private void applyIdleState() {
    // Only the X-button idle mode should keep the shooter spun up persistently.
    if (spinUp50Requested) {
      enableShooterAtSpeed(SHOOTER_TARGET_SPEED_SPINUP50_RPM);
    } else {
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
              loaderMotor.set(atSpeed ? LAUNCHING_LOADER_OUTPUT : 0.0);
            })
            .finallyDo(interrupted -> {
              intakeMotor.setVoltage(0.0);
              loaderMotor.set(0.0);
              if (!spinUp50Requested) {
                disableShooter();
              }
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
          if (!spinUp50Requested) {
            disableShooter();
          }
        });
  }

  /**
   * Intake from floor/storage without spinning up the shooter.
   * Runs intake + loader (indexer) only; shooter stays off unless the X-toggle
   * idle mode
   * is requested, in which case it is restored after the command ends.
   *
   * <p>
   * Both motors are set every cycle in execute() so the loader runs continuously
   * and redirects fuel into storage. The command never self-finishes; it runs
   * until
   * the trigger is released or another command requiring this subsystem
   * interrupts.
   */
  public Command commandIntake() {
    return Commands.run(
        () -> {
          disableShooter();
          intakeMotor.setVoltage(INTAKING_INTAKE_OUTPUT);
          loaderMotor.set(INTAKING_LOADER_OUTPUT);
        },
        this)
        .withName("Intake")
        .finallyDo(interrupted -> {
          intakeMotor.setVoltage(0.0);
          loaderMotor.set(0.0);
          if (spinUp50Requested) {
            enableShooterAtSpeed(SHOOTER_TARGET_SPEED_SPINUP50_RPM);
          } else {
            disableShooter();
          }
        });
  }

  /**
   * Shoot: spin up shooter using encoder control, then feed intake + loader.
   * Loader is gated on shooter speed so balls only feed when near target RPM.
   * When released, shooter only continues if the X-button idle spin-up is
   * enabled.
   */
  public Command commandLaunch() {
    return Commands.sequence(
        // Spin up shooter first using encoder control.
        this.run(() -> enableShooterAtSpeed(SHOOTER_TARGET_SPEED_LAUNCH_RPM))
            .withTimeout(INTAKE_SPIN_UP_SECONDS),
        // Then run shooter at speed while feeding intake and loader.
        this.run(
            () -> {
              enableShooterAtSpeed(SHOOTER_TARGET_SPEED_LAUNCH_RPM);
              // Only feed balls once shooter is near target speed.
              double current = getShooterSpeedRpm();
              boolean atSpeed = current >= SHOOTER_TARGET_SPEED_LAUNCH_RPM * SHOOTER_SPINUP_THRESHOLD_FRACTION;
              loaderMotor.set(atSpeed ? LAUNCHING_LOADER_OUTPUT : 0.0);
            })
            .finallyDo(interrupted -> {
              // Stop loader when command ends; shooter only stays on if the
              // X-button spin-up mode is enabled (persistent spin-up).
              loaderMotor.set(0.0);
              if (!spinUp50Requested) {
                disableShooter();
              }
            }));
  }

  /**
   * High-speed shoot: similar to {@link #commandLaunch()} but with a higher
   * shooter RPM for a stronger shot.
   */
  public Command commandHighSpeedLaunch() {
    final double highSpeedRpm = SHOOTER_TARGET_SPEED_HIGH_RPM;
    return Commands.sequence(
        this.run(() -> enableShooterAtSpeed(highSpeedRpm))
            .withTimeout(INTAKE_SPIN_UP_SECONDS),
        this.run(
            () -> {
              enableShooterAtSpeed(highSpeedRpm);
              double current = getShooterSpeedRpm();
              boolean atSpeed =
                  current >= highSpeedRpm * SHOOTER_SPINUP_THRESHOLD_FRACTION;
              loaderMotor.set(atSpeed ? LAUNCHING_LOADER_OUTPUT : 0.0);
            })
            .finallyDo(interrupted -> {
              loaderMotor.set(0.0);
              if (!spinUp50Requested) {
                disableShooter();
              }
            }));
  }

  /**
   * Ultra-speed shoot: highest shooter RPM for long-range shots.
   */
  public Command commandUltraSpeedLaunch() {
    final double ultraRpm = SHOOTER_TARGET_SPEED_ULTRA_RPM;
    return Commands.sequence(
        this.run(() -> enableShooterAtSpeed(ultraRpm))
            .withTimeout(INTAKE_SPIN_UP_SECONDS),
        this.run(
            () -> {
              enableShooterAtSpeed(ultraRpm);
              double current = getShooterSpeedRpm();
              boolean atSpeed =
                  current >= ultraRpm * SHOOTER_SPINUP_THRESHOLD_FRACTION;
              loaderMotor.set(atSpeed ? LAUNCHING_LOADER_OUTPUT : 0.0);
            })
            .finallyDo(interrupted -> {
              loaderMotor.set(0.0);
              if (!spinUp50Requested) {
                disableShooter();
              }
            }));
  }

  /**
   * Intake pulsing helper: repeated until the command is interrupted.
   * Uses INTAKE_PULSE_ON_SECONDS and INTAKE_PULSE_OFF_SECONDS from Constants.
   */
  public Command commandIntakePulse() {
    return Commands.repeatingSequence(
            Commands.run(
                    () -> intakeMotor.setVoltage(INTAKING_INTAKE_OUTPUT))
                .withTimeout(INTAKE_PULSE_ON_SECONDS),
            Commands.run(
                    () -> intakeMotor.setVoltage(0.0))
                .withTimeout(INTAKE_PULSE_OFF_SECONDS))
        .finallyDo(interrupted -> intakeMotor.setVoltage(0.0));
  }

  /** Eject: IO off; intake and loader run in reverse. */
  public Command commandEject() {
    // Reverse intake and loader relative to normal intaking so game pieces exit.
    return commandSpeeds(0, -INTAKING_INTAKE_OUTPUT, -INTAKING_LOADER_OUTPUT);
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
