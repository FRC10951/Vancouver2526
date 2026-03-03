package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Commands;

import static frc.robot.Constants.IoConstants.*;
import frc.robot.Constants.IoConstants.IoCanIdGroup;

/**
 * Fuel system: IO motor (CAN 9), intake (CAN 12), and loader (CAN 19), all
 * brushless SPARK MAX.
 */
public class IoSubsystem extends SubsystemBase {
  private final IoCanIdGroup canIds;
  private final SparkMax ioMotor;
  private final SparkMax intakeMotor;
  private final SparkMax loaderMotor;

  public IoSubsystem() {
    this(IO_CAN_IDS);
  }

  public IoSubsystem(IoCanIdGroup canIds) {
    this.canIds = canIds;

    // Flywheel (CAN 9) is brushless; intake/loader are brushed.
    ioMotor = new SparkMax(canIds.ioMotorId, MotorType.kBrushless);
    SparkMaxConfig ioConfig = new SparkMaxConfig();
    ioConfig.smartCurrentLimit(IO_MOTOR_CURRENT_LIMIT);
    ioMotor.configure(ioConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    intakeMotor = new SparkMax(canIds.intakeMotorId, MotorType.kBrushed);
    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    intakeConfig.smartCurrentLimit(INTAKE_MOTOR_CURRENT_LIMIT);
    intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    loaderMotor = new SparkMax(canIds.loaderMotorId, MotorType.kBrushed);
    SparkMaxConfig loaderConfig = new SparkMaxConfig();
    loaderConfig.smartCurrentLimit(LOADER_MOTOR_CURRENT_LIMIT);
    loaderMotor.configure(loaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** Set IO and intake motors by voltage and loader by duty cycle (0–1). */
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

  /**
   * Intake from floor/storage into the robot.
   *
   * <p>Behavior:
   * - IO flywheel (CAN 9) is OFF.
   * - Intake motor (CAN 12) pulls fuel in.
   * - Loader (CAN 19) runs opposite the launch direction to move fuel toward the intake.
   */
  public Command commandIntake() {
    return commandSpeeds(INTAKING_IO_VOLTAGE, INTAKING_INTAKE_OUTPUT, INTAKING_LOADER_OUTPUT);
  }

  /** Spin up / prepare without fully launching (optional helper). */
  public Command commandPrepare() {
    return commandSpeeds(PREPARING_IO_VOLTAGE, INTAKING_INTAKE_OUTPUT, PREPARING_LOADER_OUTPUT);
  }

  /**
   * Launch fuel toward the target.
   *
   * <p>Behavior:
   * <ol>
   *   <li>Spin up flywheel (CAN 9) for 0.5 s with no feeding.</li>
   *   <li>Then continue spinning flywheel and run the loader (CAN 19) in the
   *       launch direction, while the command is held.</li>
   * </ol>
   * The floor intake motor does not run while launching.
   */
  public Command commandLaunch() {
    Command spinUp =
        runEnd(
            () -> setSpeeds(LAUNCHING_IO_VOLTAGE, 0.0, 0.0),
            this::stop);

    Command feed =
        runEnd(
            () -> setSpeeds(LAUNCHING_IO_VOLTAGE, 0.0, LAUNCHING_LOADER_OUTPUT),
            this::stop);

    return Commands.sequence(spinUp.withTimeout(0.5), feed);
  }

  /** Eject fuel back out the intake (reverse of intake preset, including flywheel). */
  public Command commandEject() {
    return commandSpeeds(-INTAKING_IO_VOLTAGE, -INTAKING_INTAKE_OUTPUT, -INTAKING_LOADER_OUTPUT);
  }
}
