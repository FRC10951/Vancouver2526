package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.IoConstants.*;

/**
 * Fuel system: IO motor (CAN 9) and loader (CAN 19), both brushed SPARK MAX.
 * Intake motor (CAN 12) is in {@link frc.robot.Constants.IoConstants}; add here
 * when wired.
 */
public class IoSubsystem extends SubsystemBase {
  private final SparkMax ioMotor;
  private final SparkMax intakeMotor;
  private final SparkMax loaderMotor;

  public IoSubsystem() {
    ioMotor = new SparkMax(IO_MOTOR_ID, MotorType.kBrushed);
    SparkMaxConfig ioConfig = new SparkMaxConfig();
    ioConfig.smartCurrentLimit(IO_MOTOR_CURRENT_LIMIT);
    ioMotor.configure(ioConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    intakeMotor = new SparkMax(INTAKE_MOTOR_ID, MotorType.kBrushed);
    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    intakeConfig.smartCurrentLimit(INTAKE_MOTOR_CURRENT_LIMIT);
    intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    loaderMotor = new SparkMax(LOADER_MOTOR_ID, MotorType.kBrushed);
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

  public Command commandSpeeds(double ioVoltage, double intakeOutput, double loaderOutput) {
    return startEnd(() -> setSpeeds(ioVoltage, intakeOutput, loaderOutput), this::stop);
  }

  /** Intake from floor/storage into the robot. */
  public Command commandIntake() {
    return commandSpeeds(INTAKING_IO_VOLTAGE, INTAKING_INTAKE_OUTPUT, INTAKING_LOADER_OUTPUT);
  }

  /** Spin up / prepare without fully launching (optional helper). */
  public Command commandPrepare() {
    return commandSpeeds(PREPARING_IO_VOLTAGE, INTAKING_INTAKE_OUTPUT, PREPARING_LOADER_OUTPUT);
  }

  /** Launch fuel toward the target. */
  public Command commandLaunch() {
    return commandSpeeds(LAUNCHING_IO_VOLTAGE, INTAKING_INTAKE_OUTPUT, LAUNCHING_LOADER_OUTPUT);
  }

  /** Eject fuel back out the intake. */
  public Command commandEject() {
    return commandSpeeds(-INTAKING_IO_VOLTAGE, -INTAKING_INTAKE_OUTPUT, -INTAKING_LOADER_OUTPUT);
  }
}
