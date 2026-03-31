package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.IoConstants.*;

import frc.robot.Constants.IoConstants.IoCanIdGroup;
import frc.robot.util.RpmPidVelocityController;

/**
   * Fuel system: Flywheel motor, intake, and loader (CAN IDs from
 * Constants.IoConstants), all
 * brushless SPARK MAX.
 */
public class IoSubsystem extends SubsystemBase {
  private final SparkMax flywheelMotor;
  private final RelativeEncoder flywheelEncoder;
  /** When true, idle state or toggle modes keep shooter spinning. */
  private boolean shooterEnabled = false;
  /** Target shooter speed in RPM for encoder-based control. */
  private double shooterTargetSpeedRpm = 0.0;
  /**
   * When true, idle state runs flywheel at 50% equivalent speed (survives
   * intake/launch).
   */
  private boolean spinUp50Requested = false;
  private final SparkMax intakeMotor;
  private final RelativeEncoder intakeEncoder;
  /** Intake closed-loop enable/target (mirrors shooter control structure). */
  private boolean intakeEnabled = false;
  private double intakeTargetSpeedRpm = 0.0;
  private final SparkMax loaderMotor;

  private final RpmPidVelocityController shooterVelocityLoop =
      new RpmPidVelocityController(
          SHOOTER_PID_KP,
          SHOOTER_PID_KI,
          SHOOTER_PID_KD,
          SHOOTER_FF_KS,
          SHOOTER_FF_KV,
          SHOOTER_FF_KA,
          SHOOTER_PID_INTEGRATOR_MAX,
          SHOOTER_MAX_VOLTAGE,
          SHOOTER_SPINUP_THRESHOLD_FRACTION);

  private final RpmPidVelocityController intakeVelocityLoop =
      new RpmPidVelocityController(
          INTAKE_PID_KP,
          INTAKE_PID_KI,
          INTAKE_PID_KD,
          INTAKE_FF_KS,
          INTAKE_FF_KV,
          INTAKE_FF_KA,
          INTAKE_PID_INTEGRATOR_MAX,
          INTAKE_MAX_VOLTAGE,
          INTAKE_SPINUP_THRESHOLD_FRACTION);

  private double shooterPidLastSetpointRpm = Double.NaN;
  private double intakePidLastSetpointRpm = Double.NaN;

  // Simple jam-detection state.
  private boolean shooterJamDetected = false;
  private boolean intakeJamDetected = false;

  /**
   * When true, {@link #updateShooterControl()} applies {@link #flywheelOpenLoopVolts} and skips PID
   * (used for jam clear where commands run before periodic would otherwise zero the motor).
   */
  private boolean flywheelOpenLoopOverride = false;

  private double flywheelOpenLoopVolts = 0.0;

  // Thresholds for jam heuristics (RPM error and time).
  private static final double SHOOTER_JAM_ERROR_RPM = 500.0;
  private static final double INTAKE_JAM_ERROR_RPM = 500.0;
  private static final double JAM_DEBOUNCE_SECONDS = 0.25;

  private double shooterJamSinceSeconds = Double.NaN;
  private double intakeJamSinceSeconds = Double.NaN;

  public IoSubsystem() {
    this(IO_CAN_IDS);
  }

  public IoSubsystem(IoCanIdGroup canIds) {
    flywheelMotor = new SparkMax(canIds.ioMotorId, MotorType.kBrushless);
    SparkMaxConfig flywheelConfig = new SparkMaxConfig();
    flywheelConfig.smartCurrentLimit(FLYWHEEL_MOTOR_CURRENT_LIMIT);
    flywheelConfig.voltageCompensation(12.0);
    flywheelMotor.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    flywheelEncoder = flywheelMotor.getEncoder();

    intakeMotor = new SparkMax(canIds.intakeMotorId, MotorType.kBrushless);
    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    intakeConfig.smartCurrentLimit(INTAKE_MOTOR_CURRENT_LIMIT);
    intakeConfig.voltageCompensation(12.0);
    intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeEncoder = intakeMotor.getEncoder();

    loaderMotor = new SparkMax(canIds.loaderMotorId, MotorType.kBrushless);
    SparkMaxConfig loaderConfig = new SparkMaxConfig();
    loaderConfig.smartCurrentLimit(LOADER_MOTOR_CURRENT_LIMIT);
    loaderMotor.configure(loaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // Keep shooter speed control running every cycle whenever it is enabled.
    updateShooterControl();
    updateIntakeControl();

    // Publish jam state for driver / pit visibility.
    SmartDashboard.putBoolean("IO/Shooter jam", shooterJamDetected);
    SmartDashboard.putBoolean("IO/Intake jam", intakeJamDetected);
  }

  /** Set flywheel motor by voltage and loader by duty cycle (0?1). */
  public void setSpeeds(double flywheelVoltage, double intakeOutput, double loaderOutput) {
    flywheelMotor.setVoltage(flywheelVoltage);
    intakeMotor.setVoltage(intakeOutput);
    loaderMotor.set(loaderOutput);
  }

  /** Returns the current shooter (flywheel motor) speed in RPM. */
  public double getShooterSpeedRpm() {
    return flywheelEncoder.getVelocity();
  }

  /** Returns the current intake motor speed in RPM. */
  public double getIntakeSpeedRpm() {
    return intakeEncoder.getVelocity();
  }

  /** Enables the shooter at the given target speed (RPM). */
  public void enableShooterAtSpeed(double targetSpeedRpm) {
    double t = Math.max(0.0, targetSpeedRpm);
    if (Double.isNaN(shooterPidLastSetpointRpm) || Math.abs(t - shooterPidLastSetpointRpm) > 0.5) {
      shooterVelocityLoop.reset();
    }
    shooterPidLastSetpointRpm = t;
    shooterEnabled = true;
    shooterTargetSpeedRpm = t;
  }

  /** Disables the shooter (used when no mode requests it). */
  public void disableShooter() {
    shooterEnabled = false;
    shooterTargetSpeedRpm = 0.0;
    shooterPidLastSetpointRpm = Double.NaN;
    shooterVelocityLoop.reset();
    flywheelMotor.setVoltage(0.0);
  }

  /** Enables the intake at the given target speed (RPM). */
  public void enableIntakeAtSpeed(double targetSpeedRpm) {
    if (Double.isNaN(intakePidLastSetpointRpm)
        || Math.abs(targetSpeedRpm - intakePidLastSetpointRpm) > 0.5) {
      intakeVelocityLoop.reset();
    }
    intakePidLastSetpointRpm = targetSpeedRpm;
    intakeEnabled = true;
    intakeTargetSpeedRpm = targetSpeedRpm;
  }

  /** Disables intake closed-loop and commands zero voltage. */
  public void disableIntake() {
    intakeEnabled = false;
    intakeTargetSpeedRpm = 0.0;
    intakePidLastSetpointRpm = Double.NaN;
    intakeVelocityLoop.reset();
    intakeMotor.setVoltage(0.0);
  }

  public void stop() {
    setSpeeds(0.0, 0.0, 0.0);
    shooterEnabled = false;
    shooterTargetSpeedRpm = 0.0;
    shooterPidLastSetpointRpm = Double.NaN;
    shooterVelocityLoop.reset();
    intakeEnabled = false;
    intakeTargetSpeedRpm = 0.0;
    intakePidLastSetpointRpm = Double.NaN;
    intakeVelocityLoop.reset();
    flywheelOpenLoopOverride = false;
    shooterJamDetected = false;
    intakeJamDetected = false;
  }

  public Command commandStop() {
    return runOnce(this::stop);
  }

  /** Toggle the "spin 50% when idle" state (X button). Survives intake/launch. */
  public void toggleSpinUp50Requested() {
    spinUp50Requested = !spinUp50Requested;
  }
  /** Default command: when no other command runs, apply idle spin logic. */
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
    disableIntake();
    loaderMotor.set(0.0);
  }

  /** Velocity PID + feedforward; spin-up is open-loop max voltage (see {@link RpmPidVelocityController}). */
  private void updateShooterControl() {
    if (flywheelOpenLoopOverride) {
      flywheelMotor.setVoltage(flywheelOpenLoopVolts);
      return;
    }
    if (!shooterEnabled || shooterTargetSpeedRpm <= 0.0) {
      shooterVelocityLoop.reset();
      flywheelMotor.setVoltage(0.0);
      shooterJamDetected = false;
      shooterJamSinceSeconds = Double.NaN;
      return;
    }

    double currentRpm = getShooterSpeedRpm();
    double volts = shooterVelocityLoop.calculateVoltage(currentRpm, shooterTargetSpeedRpm);
    if (Double.isNaN(volts)) {
      flywheelMotor.setVoltage(Math.copySign(SHOOTER_MAX_VOLTAGE, shooterTargetSpeedRpm));
    } else {
      flywheelMotor.setVoltage(volts);
    }

    // Jam heuristic: large sustained RPM error in the commanded direction.
    double error = shooterTargetSpeedRpm - currentRpm;
    boolean errorLargeAndSameSign =
        Math.abs(error) > SHOOTER_JAM_ERROR_RPM && Math.signum(error) == Math.signum(shooterTargetSpeedRpm);
    updateShooterJamState(errorLargeAndSameSign);

    if (IO_PID_TELEMETRY) {
      SmartDashboard.putNumber("IO/Shooter RPM", currentRpm);
      SmartDashboard.putNumber("IO/Shooter RPM target", shooterTargetSpeedRpm);
      SmartDashboard.putNumber("IO/Shooter RPM err", shooterTargetSpeedRpm - currentRpm);
      // Sanity check: if commanding reverse RPM, flag if measured sign disagrees.
      boolean signMismatch =
          shooterTargetSpeedRpm < 0.0 && currentRpm > 0.0;
      SmartDashboard.putBoolean("IO/Warning/Shooter sign mismatch", signMismatch);
    }
  }

  private void updateIntakeControl() {
    if (!intakeEnabled || intakeTargetSpeedRpm == 0.0) {
      intakeVelocityLoop.reset();
      intakeMotor.setVoltage(0.0);
      intakeJamDetected = false;
      intakeJamSinceSeconds = Double.NaN;
      return;
    }

    double currentRpm = getIntakeSpeedRpm();
    double volts = intakeVelocityLoop.calculateVoltage(currentRpm, intakeTargetSpeedRpm);
    if (Double.isNaN(volts)) {
      intakeMotor.setVoltage(Math.copySign(INTAKE_MAX_VOLTAGE, intakeTargetSpeedRpm));
    } else {
      intakeMotor.setVoltage(volts);
    }

    // Jam heuristic: intake target is usually negative; look for large error with same sign.
    double error = intakeTargetSpeedRpm - currentRpm;
    boolean errorLargeAndSameSign =
        Math.abs(error) > INTAKE_JAM_ERROR_RPM && Math.signum(error) == Math.signum(intakeTargetSpeedRpm);
    updateIntakeJamState(errorLargeAndSameSign);

    if (IO_PID_TELEMETRY) {
      SmartDashboard.putNumber("IO/Intake RPM", currentRpm);
      SmartDashboard.putNumber("IO/Intake RPM target", intakeTargetSpeedRpm);
      SmartDashboard.putNumber("IO/Intake RPM err", intakeTargetSpeedRpm - currentRpm);
      // Sanity check: intake target sign should generally match INTAKE_TARGET_SPEED_RPM.
      boolean signMismatch =
          Math.signum(intakeTargetSpeedRpm) != Math.signum(INTAKE_TARGET_SPEED_RPM);
      SmartDashboard.putBoolean("IO/Warning/Intake sign mismatch", signMismatch);
    }
  }

  private void updateShooterJamState(boolean candidateJam) {
    double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    if (!candidateJam) {
      shooterJamDetected = false;
      shooterJamSinceSeconds = Double.NaN;
      return;
    }
    if (Double.isNaN(shooterJamSinceSeconds)) {
      shooterJamSinceSeconds = now;
      return;
    }
    shooterJamDetected = (now - shooterJamSinceSeconds) >= JAM_DEBOUNCE_SECONDS;
  }

  private void updateIntakeJamState(boolean candidateJam) {
    double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    if (!candidateJam) {
      intakeJamDetected = false;
      intakeJamSinceSeconds = Double.NaN;
      return;
    }
    if (Double.isNaN(intakeJamSinceSeconds)) {
      intakeJamSinceSeconds = now;
      return;
    }
    intakeJamDetected = (now - intakeJamSinceSeconds) >= JAM_DEBOUNCE_SECONDS;
  }

  public Command commandSpeeds(double flywheelVoltage, double intakeOutput, double loaderOutput) {
    return startEnd(
        () -> {
          // For legacy callers that still pass a flywheel voltage, treat that as enabling
          // shooter at the intake target speed; flywheel voltage itself is now controlled
          // by the encoder loop.
          if (flywheelVoltage != 0.0) {
            enableShooterAtSpeed(SHOOTER_TARGET_SPEED_INTAKE_RPM);
          }
          disableIntake();
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
              enableIntakeAtSpeed(INTAKE_TARGET_SPEED_RPM);
              double current = getShooterSpeedRpm();
              boolean atSpeed = current >= SHOOTER_TARGET_SPEED_INTAKE_RPM * SHOOTER_SPINUP_THRESHOLD_FRACTION;
              loaderMotor.set(atSpeed ? LAUNCHING_LOADER_OUTPUT : 0.0);
            })
            .finallyDo(interrupted -> {
              disableIntake();
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
          enableIntakeAtSpeed(INTAKE_TARGET_SPEED_RPM);
          double current = getShooterSpeedRpm();
          boolean atSpeed = current >= SHOOTER_TARGET_SPEED_INTAKE_RPM * SHOOTER_SPINUP_THRESHOLD_FRACTION;
          loaderMotor.set(atSpeed ? PREPARING_LOADER_OUTPUT : 0.0);
        }).finallyDo(interrupted -> {
          disableIntake();
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
    Command base =
        Commands.run(
            () -> {
              disableShooter();
              enableIntakeAtSpeed(INTAKE_TARGET_SPEED_RPM);
              loaderMotor.set(INTAKING_LOADER_OUTPUT);
            },
            this);

    // If intake jam is detected while intaking, briefly reverse to clear it.
    Command unjam =
        this.run(
                () -> {
                  if (intakeJamDetected) {
                    intakeMotor.setVoltage(-INTAKING_INTAKE_OUTPUT);
                    loaderMotor.set(-INTAKING_LOADER_OUTPUT);
                  }
                })
            .withTimeout(0.3);

    Command wrapped =
        base
            .until(() -> intakeJamDetected)
            .andThen(unjam)
            .andThen(
                () ->
                    SmartDashboard.putString(
                        "IO/Last jam action", "Intake jam: auto clear sequence executed"));

    return wrapped
        .withName("Intake")
        .finallyDo(interrupted -> {
          disableIntake();
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
                  boolean atSpeed =
                      current
                          >= SHOOTER_TARGET_SPEED_LAUNCH_RPM
                              * SHOOTER_SPINUP_THRESHOLD_FRACTION;
                  loaderMotor.set(atSpeed ? LAUNCHING_LOADER_OUTPUT : 0.0);
                  if (atSpeed) {
                    enableIntakeAtSpeed(INTAKE_TARGET_SPEED_RPM);
                  } else {
                    disableIntake();
                  }
                })
            .until(() -> shooterJamDetected || intakeJamDetected)
            .andThen(commandEject().withTimeout(0.4))
            .andThen(
                () ->
                    SmartDashboard.putString(
                        "IO/Last jam action", "Launch jam: auto eject sequence executed"))
            .finallyDo(interrupted -> {
              // Stop loader and intake when command ends; shooter only stays on if the
              // X-button spin-up mode is enabled (persistent spin-up).
              loaderMotor.set(0.0);
              disableIntake();
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
              if (atSpeed) {
                enableIntakeAtSpeed(INTAKE_TARGET_SPEED_RPM);
              } else {
                disableIntake();
              }
            })
            .finallyDo(interrupted -> {
              loaderMotor.set(0.0);
              disableIntake();
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
              if (atSpeed) {
                enableIntakeAtSpeed(INTAKE_TARGET_SPEED_RPM);
              } else {
                disableIntake();
              }
            })
            .finallyDo(interrupted -> {
              loaderMotor.set(0.0);
              disableIntake();
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
    edu.wpi.first.wpilibj.Timer timer = new edu.wpi.first.wpilibj.Timer();
    return Commands.run(
        () -> {
          disableIntake();
          double time = timer.get();
          double cycleTime = INTAKE_PULSE_ON_SECONDS + INTAKE_PULSE_OFF_SECONDS;
          double currentCycleTime = time % cycleTime;
          
          if (currentCycleTime < INTAKE_PULSE_ON_SECONDS) {
            intakeMotor.setVoltage(INTAKING_INTAKE_OUTPUT);
          } else {
            intakeMotor.setVoltage(0.0);
          }
        })
        .beforeStarting(timer::restart)
        .finallyDo(interrupted -> disableIntake());
  }

  /** Eject: flywheel off; intake and loader run in reverse. */
  public Command commandEject() {
    // Keep commanding reverse outputs while held so default/periodic logic
    // does not immediately zero the motors.
    return this.run(
        () -> {
          disableShooter();
          disableIntake();
          intakeMotor.setVoltage(-INTAKING_INTAKE_OUTPUT);
          loaderMotor.set(-INTAKING_LOADER_OUTPUT);
        }).finallyDo(interrupted -> {
          disableIntake();
          loaderMotor.set(0.0);
          if (spinUp50Requested) {
            enableShooterAtSpeed(SHOOTER_TARGET_SPEED_SPINUP50_RPM);
          } else {
            disableShooter();
          }
        });
  }

  /** Flywheel motor only at 50% (for X button toggle). */
  public Command commandFlywheelSpinUp50() {
    return this.run(() -> enableShooterAtSpeed(SHOOTER_TARGET_SPEED_SPINUP50_RPM));
  }

  public Command commandMaxSpin() {
    return this.run(() -> enableShooterAtSpeed(SHOOTER_TARGET_SPEED_INTAKE_RPM * 1.2));
  }

  /** Reverse flywheel and run loader (Y button). */
  public Command commandReverseFlywheelAndLoader() {
    return this.run(
        () -> {
          disableShooter();
          flywheelOpenLoopOverride = true;
          flywheelOpenLoopVolts = -SHOOTER_MAX_VOLTAGE / 2.0;
          loaderMotor.set(INTAKING_LOADER_OUTPUT);
          disableIntake();
          intakeMotor.setVoltage(INTAKING_INTAKE_OUTPUT); // Intake active whenever loader (CAN 19) is active
        }).finallyDo(interrupted -> {
          flywheelOpenLoopOverride = false;
          flywheelMotor.setVoltage(0.0);
          loaderMotor.set(0.0);
          disableIntake();
        });
  }
}
