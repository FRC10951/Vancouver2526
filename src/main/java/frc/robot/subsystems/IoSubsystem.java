package frc.robot.subsystems;

import java.io.FileWriter;

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

  // #region agent log
  private static void agentDebugLog(
      String location,
      String message,
      String hypothesisId,
      double ioVoltage,
      double intakeOutput,
      double loaderOutput) {
    try (FileWriter fw = new FileWriter("c:\\Users\\SSIS\\Downloads\\Vancouver2526\\.cursor\\debug.log", true)) {
      long ts = System.currentTimeMillis();
      String safeMessage = message.replace("\"", "\\\"");
      String json = String.format(
          "{\"id\":\"log_%d\",\"timestamp\":%d,\"runId\":\"initial\",\"hypothesisId\":\"%s\",\"location\":\"%s\",\"message\":\"%s\",\"data\":{\"ioVoltage\":%.3f,\"intakeOutput\":%.3f,\"loaderOutput\":%.3f}}\n",
          ts,
          ts,
          hypothesisId,
          location,
          safeMessage,
          ioVoltage,
          intakeOutput,
          loaderOutput);
      fw.write(json);
    } catch (Exception e) {
      // swallow any logging errors
    }
  }
  // #endregion

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

  /** Set IO and intake motors by voltage and loader by duty cycle (0–1). */
  public void setSpeeds(double ioVoltage, double intakeOutput, double loaderOutput) {
    agentDebugLog(
        "IoSubsystem.java:50",
        "setSpeeds invoked",
        "A",
        ioVoltage,
        intakeOutput,
        loaderOutput);
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
   * <p>
   * Behavior:
   * - IO flywheel (CAN 9) is OFF.
   * - Intake motor (CAN 12) pulls fuel in.
   * - Loader (CAN 19) runs opposite the launch direction to move fuel toward the
   * intake.
   */
  public Command commandIntake() {
    // Only floor intake (CAN 12) and hopper/loader (CAN 19) run while intaking.
    // Use the loader direction that moves balls from floor toward the hopper.
    return commandSpeeds(0.0, INTAKING_INTAKE_OUTPUT, -INTAKING_LOADER_OUTPUT);
  }

  /** Spin up / prepare shooter without fully launching (optional helper). */
  public Command commandPrepare() {
    // Shooter only, no intake or loader.
    return commandSpeeds(PREPARING_IO_VOLTAGE, 0.0, PREPARING_LOADER_OUTPUT);
  }

  /**
   * Launch fuel toward the target.
   *
   * <p>
   * Behavior:
   * <ol>
   * <li>Spin up flywheel (CAN 9) for 0.5 s with no feeding.</li>
   * <li>Then continue spinning flywheel and run the loader (CAN 19) in the
   * launch direction, while the command is held.</li>
   * </ol>
   * The floor intake motor does not run while launching.
   */
  public Command commandLaunch() {
    Command spinUp = runEnd(
        () -> setSpeeds(LAUNCHING_IO_VOLTAGE, 0.0, 0.0),
        this::stop);

    Command feed = runEnd(
        () -> setSpeeds(LAUNCHING_IO_VOLTAGE, 0.0, LAUNCHING_LOADER_OUTPUT),
        this::stop);

    return Commands.sequence(spinUp.withTimeout(0.5), feed);
  }

  /**
   * Eject fuel back out the intake (reverse of intake preset, including
   * flywheel).
   */
  public Command commandEject() {
    return commandSpeeds(-INTAKING_IO_VOLTAGE, -INTAKING_INTAKE_OUTPUT, -INTAKING_LOADER_OUTPUT);
  }

  /** Toggleable flywheel-only command (CAN 9 on/off). */
  public Command commandFlywheelToggle() {
    return runEnd(
        () -> ioMotor.setVoltage(LAUNCHING_IO_VOLTAGE),
        () -> ioMotor.setVoltage(0.0));
  }
}
