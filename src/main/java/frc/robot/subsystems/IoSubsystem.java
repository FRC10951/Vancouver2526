package frc.robot.subsystems;

import com.revrobotics.PersistMode;
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
  /** When true, idle state runs IO at 50% (survives intake/launch). */
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
    ioMotor.configure(ioConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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

  public void stop() {
    setSpeeds(0.0, 0.0, 0.0);
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
    if (spinUp50Requested) {
      setSpeeds(IO_SPIN_UP_50_VOLTAGE, 0, 0);
    } else {
      stop();
    }
  }

  public Command commandSpeeds(double ioVoltage, double intakeOutput, double loaderOutput) {
    return startEnd(() -> setSpeeds(ioVoltage, intakeOutput, loaderOutput), this::stop);
  }

  /**
   * Intake from floor/storage into the robot. IO spins up first, then
   * intake/loader.
   */
  public Command commandIntake() {
    return Commands.sequence(
        commandSpeeds(INTAKING_IO_VOLTAGE, 0, 0).withTimeout(INTAKE_SPIN_UP_SECONDS),
        commandSpeeds(INTAKING_IO_VOLTAGE, INTAKING_INTAKE_OUTPUT, INTAKING_LOADER_OUTPUT));

  }

  /** Spin up / prepare without fully launching (optional helper). */
  public Command commandPrepare() {
    return commandSpeeds(PREPARING_IO_VOLTAGE, INTAKING_INTAKE_OUTPUT, PREPARING_LOADER_OUTPUT);
  }

  /** Launch fuel toward the target (no spin-up delay). */
  public Command commandLaunch() {
    return commandSpeeds(LAUNCHING_IO_VOLTAGE, INTAKING_INTAKE_OUTPUT, LAUNCHING_LOADER_OUTPUT);
  }

  /** Eject: IO off; intake and loader run in reverse. */
  public Command commandEject() {
    return commandSpeeds(0, -INTAKING_INTAKE_OUTPUT, INTAKING_LOADER_OUTPUT);
  }

  /** IO motor only at 50% (for X button toggle). */
  public Command commandIoSpinUp50() {
    return commandSpeeds(IO_SPIN_UP_50_VOLTAGE, 0, 0);
  }

  public Command commandMaxSpin() {
    return commandSpeeds(12, 0, 0);
  }

  /** Reverse flywheel and run loader (Y button). */
  public Command commandReverseFlywheelAndLoader() {
    return commandSpeeds(-INTAKING_IO_VOLTAGE, 0, INTAKING_LOADER_OUTPUT);
  }
}
