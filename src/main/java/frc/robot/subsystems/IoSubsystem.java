// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MatchReadiness;
import frc.robot.logging.RobotTelemetryLog;
import frc.robot.util.IoControlMath;
import frc.robot.util.SparkMaxFaultReporter;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.IoConstants.*;

import frc.robot.Constants;
import frc.robot.Constants.IoConstants.IoCanIdGroup;

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

  private final SparkMaxFaultReporter flywheelFaultReporter =
      new SparkMaxFaultReporter("IO/Flywheel");
  private final SparkMaxFaultReporter intakeFaultReporter =
      new SparkMaxFaultReporter("IO/Intake");
  private final SparkMaxFaultReporter loaderFaultReporter =
      new SparkMaxFaultReporter("IO/Loader");

  private SparkMaxSim flywheelSim;
  private SparkMaxSim intakeSim;
  private SparkMaxSim loaderSim;
  private FlywheelSim shooterPlant;
  private FlywheelSim intakePlant;
  private FlywheelSim loaderPlant;

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

    double shooterIMax = SHOOTER_MAX_VOLTAGE / Math.max(SHOOTER_KI, 1e-9);
    shooterVelocityPid.setIntegratorRange(-shooterIMax, shooterIMax);
    double intakeIMax = INTAKE_MAX_VOLTAGE / Math.max(INTAKE_KI, 1e-9);
    intakeVelocityPid.setIntegratorRange(-intakeIMax, intakeIMax);

    SmartDashboard.putData("IO/Shooter velocity PID", shooterVelocityPid);
    SmartDashboard.putData("IO/Intake velocity PID", intakeVelocityPid);

    if (RobotBase.isSimulation()) {
      flywheelSim = new SparkMaxSim(flywheelMotor, DCMotor.getNEO(1));
      shooterPlant =
          new FlywheelSim(
              LinearSystemId.createFlywheelSystem(
                  DCMotor.getNEO(1), Constants.SimulationConstants.SHOOTER_FLYWHEEL_J_KG_M2, 1.0),
              DCMotor.getNEO(1));
      intakeSim = new SparkMaxSim(intakeMotor, DCMotor.getNeo550(1));
      intakePlant =
          new FlywheelSim(
              LinearSystemId.createFlywheelSystem(
                  DCMotor.getNeo550(1), Constants.SimulationConstants.INTAKE_ROLLER_J_KG_M2, 1.0),
              DCMotor.getNeo550(1));
      loaderSim = new SparkMaxSim(loaderMotor, DCMotor.getNeo550(1));
      loaderPlant =
          new FlywheelSim(
              LinearSystemId.createFlywheelSystem(
                  DCMotor.getNeo550(1), Constants.SimulationConstants.LOADER_ROLLER_J_KG_M2, 1.0),
              DCMotor.getNeo550(1));
    }
  }

  @Override
  public void periodic() {
    updateShooterControl();
    updateIntakeControl();

    flywheelFaultReporter.reportPeriodic(flywheelMotor);
    intakeFaultReporter.reportPeriodic(intakeMotor);
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
   * Desktop simulation: advance WPILib {@link FlywheelSim} plants and REV
   * {@link SparkMaxSim} so encoder RPMs react to voltage commands. No-op on the
   * roboRIO.
   */
  public void simulationPeriodic() {
    if (flywheelSim == null) {
      return;
    }
    double dt = TimedRobot.kDefaultPeriod;
    double vbus = RobotController.getBatteryVoltage();

    shooterPlant.setInputVoltage(flywheelMotor.getAppliedOutput() * vbus);
    shooterPlant.update(dt);
    flywheelSim.iterate(shooterPlant.getAngularVelocityRPM(), vbus, dt);

    intakePlant.setInputVoltage(intakeMotor.getAppliedOutput() * vbus);
    intakePlant.update(dt);
    intakeSim.iterate(intakePlant.getAngularVelocityRPM(), vbus, dt);

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
    return flywheelEncoder.getVelocity();
  }

  /** Returns the current intake motor speed in RPM. */
  public double getIntakeSpeedRpm() {
    return intakeEncoder.getVelocity();
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
}
