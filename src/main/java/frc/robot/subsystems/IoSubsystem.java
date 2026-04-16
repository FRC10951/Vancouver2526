// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MatchReadiness;
import frc.robot.logging.RobotTelemetryLog;
import frc.robot.util.IoControlMath;
import frc.robot.util.SparkMaxFaultReporter;

import static edu.wpi.first.units.Units.RPM;

import static frc.robot.Constants.IoConstants.*;

import frc.robot.Constants;
import frc.robot.Constants.IoConstants.IoCanIdGroup;

/**
 * Fuel system: flywheel and intake use Kraken X60 motors on Talon FX; loader
 * remains a brushless SPARK MAX (CAN ID from {@link Constants.IoConstants}).
 *
 * <p>Phoenix "firmware / stale CAN" spam means no Talon was reached at the
 * configured IDs and bus; fix wiring, IDs, v6 firmware, and device type before
 * tuning software.
 */
public class IoSubsystem extends SubsystemBase {
  private final TalonFX flywheelMotor;
  /** When true, idle state or toggle modes keep shooter spinning. */
  private boolean shooterEnabled = false;
  /** Target shooter speed in RPM for encoder-based control. */
  private double shooterTargetSpeedRpm = 0.0;
  /**
   * When true, idle state runs flywheel at 50% equivalent speed (survives
   * intake/launch).
   */
  private boolean spinUp50Requested = false;
  private final TalonFX intakeMotor;
  /** Intake closed-loop enable/target (mirrors shooter control structure). */
  private boolean intakeEnabled = false;
  private double intakeTargetSpeedRpm = 0.0;
  private final SparkMax loaderMotor;

  private final PIDController shooterVelocityPid =
      new PIDController(SHOOTER_KP, SHOOTER_KI, SHOOTER_KD);
  private final PIDController intakeVelocityPid =
      new PIDController(INTAKE_KP, INTAKE_KI, INTAKE_KD);
  private final SimpleMotorFeedforward shooterFeedforward =
      new SimpleMotorFeedforward(
          SHOOTER_KS_VOLTS,
          SHOOTER_VELOCITY_FF_VOLTS_PER_RPM,
          SHOOTER_KA_VOLTS_PER_RPM_PER_S);
  private final SimpleMotorFeedforward intakeFeedforward =
      new SimpleMotorFeedforward(
          INTAKE_KS_VOLTS,
          INTAKE_VELOCITY_FF_VOLTS_PER_RPM,
          INTAKE_KA_VOLTS_PER_RPM_PER_S);

  private final StickyFaultTracker flywheelStickyFaults = new StickyFaultTracker();
  private final StickyFaultTracker intakeStickyFaults = new StickyFaultTracker();
  private final SparkMaxFaultReporter loaderFaultReporter =
      new SparkMaxFaultReporter("IO/Loader");

  private TalonFXSimState flywheelSimState;
  private TalonFXSimState intakeSimState;
  private SparkMaxSim loaderSim;
  private FlywheelSim shooterPlant;
  private FlywheelSim intakePlant;
  private FlywheelSim loaderPlant;

  /**
   * Updated once per {@link #periodic()} after a single CAN refresh, so we do not
   * issue many rotor-velocity refreshes per tick (helps CAN utilization).
   */
  private double cachedFlywheelRpm;

  private double cachedIntakeRpm;

  public IoSubsystem() {
    this(IO_CAN_IDS);
  }

  public IoSubsystem(IoCanIdGroup canIds) {
    flywheelMotor = newTalonFx(canIds.ioMotorId, canIds.talonFxCanBus);
    applyTalonFxIoDefaults(flywheelMotor, FLYWHEEL_MOTOR_CURRENT_LIMIT);

    intakeMotor = newTalonFx(canIds.intakeMotorId, canIds.talonFxCanBus);
    applyTalonFxIoDefaults(intakeMotor, INTAKE_MOTOR_CURRENT_LIMIT);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, flywheelMotor.getRotorVelocity(), intakeMotor.getRotorVelocity());

    loaderMotor = new SparkMax(canIds.loaderMotorId, MotorType.kBrushless);
    SparkMaxConfig loaderConfig = new SparkMaxConfig();
    loaderConfig.smartCurrentLimit(LOADER_MOTOR_CURRENT_LIMIT);
    loaderMotor.configure(loaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    double shooterIMax = SHOOTER_MAX_VOLTAGE / Math.max(SHOOTER_KI, 1e-9);
    shooterVelocityPid.setIntegratorRange(-shooterIMax, shooterIMax);
    double intakeIMax = INTAKE_MAX_VOLTAGE / Math.max(INTAKE_KI, 1e-9);
    intakeVelocityPid.setIntegratorRange(-intakeIMax, intakeIMax);

    SmartDashboard.putData("IO/Shooter velocity PID", shooterVelocityPid);
    SmartDashboard.putData("IO/Intake velocity PID", intakeVelocityPid);

    if (RobotBase.isSimulation()) {
      DCMotor kraken = DCMotor.getKrakenX60Foc(1);
      flywheelSimState = flywheelMotor.getSimState();
      flywheelSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
      shooterPlant =
          new FlywheelSim(
              LinearSystemId.createFlywheelSystem(
                  kraken, Constants.SimulationConstants.SHOOTER_FLYWHEEL_J_KG_M2, 1.0),
              kraken);
      intakeSimState = intakeMotor.getSimState();
      intakeSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
      intakePlant =
          new FlywheelSim(
              LinearSystemId.createFlywheelSystem(
                  kraken, Constants.SimulationConstants.INTAKE_ROLLER_J_KG_M2, 1.0),
              kraken);
      loaderSim = new SparkMaxSim(loaderMotor, DCMotor.getNeo550(1));
      loaderPlant =
          new FlywheelSim(
              LinearSystemId.createFlywheelSystem(
                  DCMotor.getNeo550(1), Constants.SimulationConstants.LOADER_ROLLER_J_KG_M2, 1.0),
              DCMotor.getNeo550(1));
    }
  }

  private static TalonFX newTalonFx(int deviceId, String canBus) {
    if (canBus == null || canBus.isEmpty()) {
      return new TalonFX(deviceId);
    }
    return new TalonFX(deviceId, new CANBus(canBus));
  }

  private static void applyTalonFxIoDefaults(TalonFX motor, int statorCurrentLimitAmps) {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    // If closed-loop RPM disagrees with mechanism direction vs. the Neo era, set
    // cfg.MotorOutput.Inverted here (or in Tuner X) and retest.
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    cfg.CurrentLimits.StatorCurrentLimit = statorCurrentLimitAmps;
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    motor.getConfigurator().apply(cfg);
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(flywheelMotor.getRotorVelocity(), intakeMotor.getRotorVelocity());
    cachedFlywheelRpm = flywheelMotor.getRotorVelocity().getValue().in(RPM);
    cachedIntakeRpm = intakeMotor.getRotorVelocity().getValue().in(RPM);

    updateShooterControl();
    updateIntakeControl();

    reportTalonFxFaults(flywheelMotor, "IO/Flywheel", flywheelStickyFaults);
    reportTalonFxFaults(intakeMotor, "IO/Intake", intakeStickyFaults);
    loaderFaultReporter.reportPeriodic(loaderMotor);

    SmartDashboard.putNumber("IO/Shooter RPM", getShooterSpeedRpm());
    SmartDashboard.putNumber(
        "IO/Shooter target RPM", shooterEnabled ? shooterTargetSpeedRpm : 0.0);
    SmartDashboard.putNumber("IO/Intake RPM", getIntakeSpeedRpm());
    SmartDashboard.putNumber(
        "IO/Intake target RPM", intakeEnabled ? intakeTargetSpeedRpm : 0.0);

    SmartDashboard.putNumber("Electrical/Battery V", MatchReadiness.getBatteryVoltage());
    SmartDashboard.putBoolean(
        "Electrical/Battery OK for shooter", MatchReadiness.isBatteryHealthyForShooter());
    SmartDashboard.putBoolean(
        "Electrical/Battery caution band", MatchReadiness.isBatteryInCautionBand());

    RobotTelemetryLog.recordIo(
        getShooterSpeedRpm(),
        shooterEnabled ? shooterTargetSpeedRpm : 0.0,
        getIntakeSpeedRpm(),
        intakeEnabled ? intakeTargetSpeedRpm : 0.0);
  }

  /**
   * Desktop simulation: advance WPILib {@link FlywheelSim} plants and device sim
   * state so encoder RPMs react to voltage commands. No-op on the roboRIO.
   */
  public void simulationPeriodic() {
    if (flywheelSimState == null) {
      return;
    }
    double dt = TimedRobot.kDefaultPeriod;
    double vbus = RobotController.getBatteryVoltage();

    flywheelSimState.setSupplyVoltage(vbus);
    shooterPlant.setInputVoltage(flywheelSimState.getMotorVoltage());
    shooterPlant.update(dt);
    flywheelSimState.setRotorVelocity(shooterPlant.getAngularVelocityRPM() / 60.0);

    intakeSimState.setSupplyVoltage(vbus);
    intakePlant.setInputVoltage(intakeSimState.getMotorVoltage());
    intakePlant.update(dt);
    intakeSimState.setRotorVelocity(intakePlant.getAngularVelocityRPM() / 60.0);

    loaderPlant.setInputVoltage(loaderMotor.getAppliedOutput() * vbus);
    loaderPlant.update(dt);
    loaderSim.iterate(loaderPlant.getAngularVelocityRPM(), vbus, dt);
  }

  /** Set flywheel motor by voltage and loader by duty cycle (0?1). */
  public void setSpeeds(double flywheelVoltage, double intakeOutput, double loaderOutput) {
    flywheelMotor.setVoltage(flywheelVoltage);
    intakeMotor.setVoltage(intakeOutput);
    loaderMotor.set(loaderOutput);
  }

  /** Returns the current shooter (flywheel motor) speed in RPM. */
  public double getShooterSpeedRpm() {
    return cachedFlywheelRpm;
  }

  /** Returns the current intake motor speed in RPM. */
  public double getIntakeSpeedRpm() {
    return cachedIntakeRpm;
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
    shooterVelocityPid.reset();
    flywheelMotor.setVoltage(0.0);
  }

  /** Enables the intake at the given target speed (RPM). */
  public void enableIntakeAtSpeed(double targetSpeedRpm) {
    intakeEnabled = true;
    intakeTargetSpeedRpm = targetSpeedRpm;
  }

  /** Disables intake closed-loop and commands zero voltage. */
  public void disableIntake() {
    intakeEnabled = false;
    intakeTargetSpeedRpm = 0.0;
    intakeVelocityPid.reset();
    intakeMotor.setVoltage(0.0);
  }

  public void stop() {
    setSpeeds(0.0, 0.0, 0.0);
    shooterEnabled = false;
    shooterTargetSpeedRpm = 0.0;
    intakeEnabled = false;
    intakeTargetSpeedRpm = 0.0;
    shooterVelocityPid.reset();
    intakeVelocityPid.reset();
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
      // Let the shooter coast while it is above the desired idle speed, and only
      // re-engage closed-loop control if it falls below the target RPM. This keeps
      // X as a "minimum speed" clamp instead of instantly forcing a new setpoint.
      double current = getShooterSpeedRpm();
      double desiredRpm = SHOOTER_TARGET_SPEED_SPINUP50_RPM;
      if (current > desiredRpm) {
        // Above idle target: coast freely (no control).
        disableShooter();
      } else {
        // Below idle target: hold at the desired RPM.
        enableShooterAtSpeed(desiredRpm);
      }
    } else {
      disableShooter();
    }
    // Intake and loader are idle in this state.
    updateShooterControl();
    disableIntake();
    loaderMotor.set(0.0);
  }

  /**
   * Closed-loop shooter speed: max voltage spin-up, then feedforward + PID on RPM.
   */
  private void updateShooterControl() {
    if (!shooterEnabled || shooterTargetSpeedRpm <= 0.0) {
      return;
    }

    double currentSpeedRpm = getShooterSpeedRpm();
    double target = shooterTargetSpeedRpm;
    shooterVelocityPid.setSetpoint(target);

    // Spin-up region: below threshold, use max voltage (no integral windup).
    if (currentSpeedRpm < target * SHOOTER_SPINUP_THRESHOLD_FRACTION) {
      shooterVelocityPid.reset();
      flywheelMotor.setVoltage(Math.copySign(SHOOTER_MAX_VOLTAGE, target));
      return;
    }

    double ff = shooterFeedforward.calculateWithVelocities(currentSpeedRpm, target);
    double commanded =
        IoControlMath.clampSymmetric(
            ff + shooterVelocityPid.calculate(currentSpeedRpm), SHOOTER_MAX_VOLTAGE);

    flywheelMotor.setVoltage(commanded);
  }

  /**
   * Closed-loop intake speed: max voltage spin-up, then feedforward + PID on RPM.
   */
  private void updateIntakeControl() {
    if (!intakeEnabled || intakeTargetSpeedRpm == 0.0) {
      return;
    }

    double currentSpeedRpm = getIntakeSpeedRpm();
    double target = intakeTargetSpeedRpm;
    intakeVelocityPid.setSetpoint(target);

    if (IoControlMath.intakeBelowSpinupFraction(
        currentSpeedRpm, target, INTAKE_SPINUP_THRESHOLD_FRACTION)) {
      intakeVelocityPid.reset();
      intakeMotor.setVoltage(Math.copySign(INTAKE_MAX_VOLTAGE, target));
      return;
    }

    double ff = intakeFeedforward.calculateWithVelocities(currentSpeedRpm, target);
    double commanded =
        IoControlMath.clampSymmetric(
            ff + intakeVelocityPid.calculate(currentSpeedRpm), INTAKE_MAX_VOLTAGE);

    if (Math.signum(commanded) != 0.0 && Math.signum(commanded) != Math.signum(target)) {
      commanded = 0.0;
    }

    intakeMotor.setVoltage(commanded);
  }

  public Command commandSpeeds(double flywheelVoltage, double intakeOutput, double loaderOutput) {
    return startEnd(
        () -> {
          // For legacy callers that still pass a flywheel voltage, treat that as enabling
          // shooter at the intake target speed; flywheel voltage itself is now controlled
          // by the encoder loop.
          if (flywheelVoltage != 0.0) {
            enableShooterAtSpeed(INTAKE_TARGET_SPEED_INTAKE_RPM);
          }
          disableIntake();
          intakeMotor.setVoltage(intakeOutput);
          loaderMotor.set(loaderOutput);
        },
        this::stop);
  }

  public Command commandIntakeAuton() {
    return Commands.sequence(
        this.run(() -> enableShooterAtSpeed(INTAKE_TARGET_SPEED_INTAKE_RPM))
            .withTimeout(INTAKE_AUTON_SPIN_UP_SECONDS),
        this.run(
            () -> {
              enableShooterAtSpeed(INTAKE_TARGET_SPEED_INTAKE_RPM);
              enableIntakeAtSpeed(INTAKE_TARGET_SPEED_RPM);
              double current = getShooterSpeedRpm();
              boolean atSpeed =
                  IoControlMath.shooterAtOrAboveFractionOfTarget(
                      current,
                      INTAKE_TARGET_SPEED_INTAKE_RPM,
                      SHOOTER_SPINUP_THRESHOLD_FRACTION);
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
          enableShooterAtSpeed(INTAKE_TARGET_SPEED_INTAKE_RPM);
          enableIntakeAtSpeed(INTAKE_TARGET_SPEED_RPM);
          double current = getShooterSpeedRpm();
          boolean atSpeed =
              IoControlMath.shooterAtOrAboveFractionOfTarget(
                  current,
                  INTAKE_TARGET_SPEED_INTAKE_RPM,
                  SHOOTER_SPINUP_THRESHOLD_FRACTION);
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
    return Commands.run(
        () -> {
          enableIntakeAtSpeed(INTAKE_TARGET_SPEED_RPM);
          loaderMotor.set(INTAKING_LOADER_OUTPUT);
        },
        this)
        .withName("Intake")
        .finallyDo(interrupted -> {
          disableIntake();
          loaderMotor.set(0.0);
        });
  }

  /**
   * Shoot: spin up shooter using encoder control, then feed intake + loader.
   * Loader is gated on shooter speed so balls only feed when near target RPM.
   * When released, shooter only continues if the X-button idle spin-up is
   * enabled.
   */
  public Command commandLaunch() {
    return createLaunchCommand(SHOOTER_TARGET_SPEED_LAUNCH_RPM);
  }

  /**
   * High-speed shoot: similar to {@link #commandLaunch()} but with a higher
   * shooter RPM for a stronger shot.
   */
  public Command commandHighSpeedLaunch() {
    return createLaunchCommand(SHOOTER_TARGET_SPEED_HIGH_RPM);
  }

  /**
   * Ultra-speed shoot: highest shooter RPM for long-range shots.
   */
  public Command commandUltraSpeedLaunch() {
    return createLaunchCommand(SHOOTER_TARGET_SPEED_ULTRA_RPM);
  }

  /**
   * Phase 1 of teleop launch: spin shooter toward {@code shooterRpm}; loader and
   * intake stay off until RPM crosses the spin-up threshold (or this phase times
   * out).
   */
  public Command commandLaunchSpinUpPhase(double shooterRpm) {
    return createLaunchSpinUpPhase(shooterRpm);
  }

  /**
   * Phase 2 of teleop launch: hold shooter setpoint and run loader + intake
   * continuously. Pair with {@link #commandLaunchSpinUpPhase(double)} when
   * another subsystem (e.g. drivetrain wiggle) must not start until spin-up.
   */
  public Command commandLaunchSustainPhase(double shooterRpm) {
    return createLaunchSustainPhase(shooterRpm);
  }

  private Command createLaunchSpinUpPhase(double shooterRpm) {
    return this.run(
            () -> {
              enableShooterAtSpeed(shooterRpm);
              double current = getShooterSpeedRpm();
              boolean atSpeed =
                  IoControlMath.shooterAtOrAboveFractionOfTarget(
                      current, shooterRpm, SHOOTER_SPINUP_THRESHOLD_FRACTION);
              if (atSpeed) {
                loaderMotor.set(LAUNCHING_LOADER_OUTPUT);
                enableIntakeAtSpeed(INTAKE_TARGET_SPEED_RPM);
              } else {
                loaderMotor.set(0.0);
                disableIntake();
              }
            })
        .withTimeout(LAUNCH_SPIN_UP_SECONDS);
  }

  private Command createLaunchSustainPhase(double shooterRpm) {
    return this.run(
            () -> {
              enableShooterAtSpeed(shooterRpm);
              loaderMotor.set(LAUNCHING_LOADER_OUTPUT);
              enableIntakeAtSpeed(INTAKE_TARGET_SPEED_RPM);
            })
        .finallyDo(interrupted -> {
          loaderMotor.set(0.0);
          disableIntake();
          if (!spinUp50Requested) {
            disableShooter();
          }
        });
  }

  /**
   * Common implementation for launch-style commands that spin up the shooter to
   * a given RPM and then conditionally feed intake + loader once at speed.
   */
  private Command createLaunchCommand(double shooterRpm) {
    return Commands.sequence(
        createLaunchSpinUpPhase(shooterRpm), createLaunchSustainPhase(shooterRpm));
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
    return this.run(() -> enableShooterAtSpeed(INTAKE_TARGET_SPEED_INTAKE_RPM * 1.2));
  }

  /** Reverse flywheel and run loader (Y button). */
  public Command commandReverseFlywheelAndLoader() {
    // Reverse shooter uses open-loop for simplicity; does not use the speed
    // controller because it is only for clearing jams.
    return this.run(
        () -> {
          disableShooter();
          flywheelMotor.setVoltage(-SHOOTER_MAX_VOLTAGE / 2.0);
          loaderMotor.set(INTAKING_LOADER_OUTPUT);
          disableIntake();
          intakeMotor.setVoltage(INTAKING_INTAKE_OUTPUT); // Intake active whenever loader (CAN 19) is active
        }).finallyDo(interrupted -> {
          flywheelMotor.setVoltage(0.0);
          loaderMotor.set(0.0);
          disableIntake();
        });
  }

  private static final class StickyFaultTracker {
    int lastStickyRawBits = -1;
  }

  private static void reportTalonFxFaults(
      TalonFX motor, String dashboardKeyPrefix, StickyFaultTracker tracker) {
    motor.getFaultField().refresh();
    motor.getStickyFaultField().refresh();
    int active = motor.getFaultField().getValue();
    int sticky = motor.getStickyFaultField().getValue();

    SmartDashboard.putNumber(dashboardKeyPrefix + "/fault raw", active);
    SmartDashboard.putString(dashboardKeyPrefix + "/faults", formatTalonFxFaultHex(active));
    SmartDashboard.putNumber(dashboardKeyPrefix + "/sticky fault raw", sticky);
    SmartDashboard.putString(dashboardKeyPrefix + "/sticky faults", formatTalonFxFaultHex(sticky));
    SmartDashboard.putBoolean(dashboardKeyPrefix + "/has active fault", active != 0);
    SmartDashboard.putBoolean(dashboardKeyPrefix + "/has sticky fault", sticky != 0);

    if (sticky != tracker.lastStickyRawBits) {
      if (sticky != 0) {
        DriverStation.reportWarning(
            "["
                + dashboardKeyPrefix
                + "] Sticky Talon FX fault(s), raw 0x"
                + Integer.toHexString(sticky),
            false);
      }
      tracker.lastStickyRawBits = sticky;
    }
  }

  private static String formatTalonFxFaultHex(int raw) {
    return raw == 0 ? "none" : ("0x" + Integer.toHexString(raw));
  }
}
