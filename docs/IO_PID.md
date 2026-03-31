# IO velocity PID — utility guide

This document describes how **shooter (flywheel)** and **intake** speed control works on this robot, what each gain does, how to tune safely, and where the code lives.

## Hardware assumption

- **Neo / Spark MAX brushless** motors on CAN with **internal encoders** (RPM feedback from `RelativeEncoder.getVelocity()`).
- The **loader** is still commanded **open-loop** (duty / voltage from `Constants`), not with PID in this project.

If feedback is wrong (inverted motor, bad units), PID cannot be reliable — fix wiring and `MotorType.kBrushless` before tuning gains.

---

## Architecture overview

| Piece | Role |
|--------|------|
| **`RpmPidVelocityController`** (`frc.robot.util`) | Wraps WPILib `PIDController` + `SimpleMotorFeedforward`. Converts RPM ↔ rotations per second (rps) for feedforward math. |
| **`IoSubsystem`** | Owns two instances (shooter + intake). Each cycle: read RPM → compute voltage → `SparkMax.setVoltage`. |
| **`Constants.IoConstants`** | All gains, limits, spin-up threshold, telemetry flag. |

**Output command:**  

\[
u = \underbrace{k_s \cdot \mathrm{sign}(\omega_t) + k_v \cdot \omega_t}_{\text{feedforward}} + \underbrace{k_p e + k_i \int e\,dt + k_d \frac{d}{dt}e}_{\text{PID}}
\]

where \(e = \omega_t - \omega_m\) in **rotations per second** (measurement and setpoint converted from RPM inside the controller).

**Spin-up:** While \(|\omega_m| < |\omega_t| \cdot \texttt{SPINUP\_THRESHOLD\_FRACTION}\), the controller returns “use open-loop max voltage” (caller applies `±MAX_VOLTAGE` with sign of target). The PID **integral is reset** in this phase to avoid **integral windup** before the wheel is near speed.

---

## PID terms (what each does)

### P — proportional (`SHOOTER_PID_KP`, `INTAKE_PID_KP`)

- Reacts to **current error** in velocity (in rps inside the loop).
- Larger **P** → faster correction, but too high → oscillation and noise amplification.
- Units in code: output **volts per (rot/s)** of error (because `PIDController` uses rps).

### I — integral (`SHOOTER_PID_KI`, `INTAKE_PID_KI`)

- Removes **steady-state error** (e.g. “always 30 RPM low” at a fixed setpoint).
- Risk: **windup** if the mechanism is saturated or stuck in spin-up — mitigated by:
  - spin-up phase (integral reset),
  - `setIntegratorRange` via **`SHOOTER_PID_INTEGRATOR_MAX` / `INTAKE_PID_INTEGRATOR_MAX`**,
  - disabling the loop when the command releases (`disableShooter` / `disableIntake` / `stop()`).

Start with **small** I; increase if you see persistent offset **after** feedforward is reasonable.

### D — derivative (`SHOOTER_PID_KD`, `INTAKE_PID_KD`)

- Dampens **rapid changes** in error. Helpful when P is aggressive.
- Encoder **velocity is noisy** at high loop rates — large **D** can inject noise into the output. Often **near zero** on flywheels; tune carefully if you add it.

---

## Feedforward terms (`SimpleMotorFeedforward`)

Feedforward predicts the voltage needed from the **setpoint** alone, so PID only corrects residual error.

| Constant | Meaning |
|----------|---------|
| **`SHOOTER_FF_KS` / `INTAKE_FF_KS`** | Static friction (volts). Often small or 0 for a spinning wheel; sometimes useful if the wheel sticks before moving. |
| **`SHOOTER_FF_KV` / `INTAKE_FF_KV`** | **Volts per rotation per second** (V·s/rotation). Main term: “how much voltage for a given steady speed.” |
| **`SHOOTER_FF_KA` / `INTAKE_FF_KA`** | Acceleration feedforward (V·s²/rotation). Used when you command changing velocity profiles; steady hold uses `calculate(velocity)` without acceleration, so **0 is OK** until you add motion profiling. |

**Rough kV estimate:** at steady state, \(k_v \approx \frac{V_{\mathrm{bus}}}{\omega_{\mathrm{free}}}\) in consistent units, where \(\omega_{\mathrm{free}}\) is free speed in **rot/s**. Example: ~12 V and ~80 rot/s → \(k_v \approx 0.15\) (order of magnitude only — **tune on the robot**).

---

## Other important constants

| Constant | Purpose |
|----------|---------|
| `SHOOTER_SPINUP_THRESHOLD_FRACTION` / `INTAKE_SPINUP_THRESHOLD_FRACTION` | Fraction of target speed below which **open-loop max voltage** is used. Higher → longer spin-up on full bus; lower → earlier switch to PID (may sag if FF is off). |
| `SHOOTER_MAX_VOLTAGE` / `INTAKE_MAX_VOLTAGE` | Output clamp (safety + hardware). |
| `SHOOTER_PID_INTEGRATOR_MAX` / `INTAKE_PID_INTEGRATOR_MAX` | Caps the PID integrator to limit windup (WPILib internal units). |
| `IO_PID_TELEMETRY` | If `true`, publishes RPM / target / error to SmartDashboard for tuning. |

---

## Reliability features in code

1. **Integral reset** when entering spin-up (`RpmPidVelocityController.calculateVoltage`).
2. **Integral reset** when disabling (`disableShooter`, `disableIntake`, `stop`).
3. **Integral reset** when the **setpoint changes** by more than ~0.5 RPM (`enableShooterAtSpeed` / `enableIntakeAtSpeed`) so switching presets (launch vs ultra) does not carry bad state.
4. **Open-loop jam / reverse** (`commandReverseFlywheelAndLoader`) sets a **`flywheelOpenLoopOverride`** flag so `periodic()` applies a fixed flywheel voltage **after** the command runs (the scheduler runs commands, then subsystem `periodic()`, which would otherwise zero the motor if only `disableShooter()` were used).
5. **Voltage compensation** on Sparks (12 V) reduces battery sag effects in hardware; PID + FF handle the rest in software.

---

## Tuning procedure (practical order)

1. **Verify sensor sign:** RPM should **increase** in the direction you expect when you command positive shooter speed. If inverted, fix motor inversion in Spark config or negate setpoints — do not “fix” with negative gains unless you know why.
2. **Feedforward first:** With **I = 0**, **D = 0**, set **P** small. Adjust **kV** (and **kS** if needed) so the wheel **roughly** reaches the target without huge error. Goal: **FF does most of the work**.
3. **Add P** to tighten tracking: increase until you see fast response without violent oscillation.
4. **Add I** only if a **steady offset** remains at constant setpoint. Increase slowly; watch for overshoot or windup.
5. **Add D** only if needed (oscillation with good P); keep small.

Use **SmartDashboard** (`IO_PID_TELEMETRY = true`) and graph **RPM**, **target**, and **error** while tuning.

---

## Where to edit

- **Gains and limits:** `Constants.IoConstants` (or the **Constants editor** GUI on the **IO / Shooter** tab).
- **Control loop implementation:** `frc.robot.util.RpmPidVelocityController`, `frc.robot.subsystems.IoSubsystem`.

---

## Related reading

- [WPILib — PIDController](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/pid.html)
- [WPILib — Feedforward](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/feedforward.html)
- Project layout: [REPO_LAYOUT.md](REPO_LAYOUT.md)
