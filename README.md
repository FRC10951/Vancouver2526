# KitBot 2026 Robot Code

WPILib command-based robot code for the **2026 KitBot** (FRC). This robot has:

- A **4-motor tank drivetrain** (CIMs on SPARK MAX, brushed) driven with **arcade drive**.
- A **fuel system** with three SPARK MAX motors defined in constants: **intake** (CAN ID 12), **IO/flywheel** (ID 9), and **loader/directional wheel** (ID 19). The `IoSubsystem` currently uses IDs 9 and 19 for intake and launch; the intake motor (12) is in constants and ready for when it is wired into the subsystem.
- A **single driver Xbox controller** (USB port 0) for drive and fuel; operator controller port is defined but not used in bindings.
- **Autonomous** options: Do Nothing, Drive Forward 2s, turn left 2s, and “drive back & shoot preload” (left or right).

This README explains how **constants** map to hardware, the **subsystems and commands**, and the **controls**.

---

## Controller layout (driver — Xbox, USB port 0)

Arcade drive runs by default. Triggers control the fuel system.

| Control | Function |
|--------|----------|
| **Left stick (Y)** | Drive forward/back (arcade). Scaling from `OperatorConstants.DRIVE_SCALING`. |
| **Right stick (X)** | Rotate left/right (arcade). Scaling from `OperatorConstants.ROTATION_SCALING`. |
| **LT** (Left trigger) | **Intake** — hold to run `IoSubsystem.commandIntake()` (IO motor 9 + loader 19 pull fuel in). |
| **RT** (Right trigger) | **Launch** — hold to run `IoSubsystem.commandLaunch()` (IO motor 9 + loader 19 launch). |
| **B** | **Reset encoders** — resets drivetrain encoders on `CANDriveSubsystem`. |

There is no operator controller in use; only the driver controller has bindings. Other fuel commands exist in code (`commandEject`, `commandPrepare`, `commandStop`) but are not bound to buttons.

---

## CAN IDs and hardware mapping

At startup, the robot prints a CAN ID list to the console and publishes it to SmartDashboard under **"CAN IDs"** (from `Constants.getCanIdsList()`).

### Drivetrain (SPARK MAX, **brushed** CIMs)

| CAN ID | Physical position | Role in code (`DriveConstants`) |
|--------|-------------------|----------------------------------|
| 11 | Left leader | `LEFT_LEADER_ID` |
| 8 | Left follower | `LEFT_FOLLOWER_ID` (follows 11) |
| 10 | Right leader | `RIGHT_LEADER_ID` |
| 7 | Right follower | `RIGHT_FOLLOWER_ID` (follows 10) |

- All four are **brushed** SPARK MAX. `CANDriveSubsystem` configures current limits, inversion (left side inverted so positive = forward), and followers. `DifferentialDrive` uses the two leaders.

### Fuel mechanism (constants and subsystem)

| CAN ID | Function | Role in code | Notes |
|--------|----------|--------------|--------|
| 9 | IO / flywheel | `IoConstants.IO_MOTOR_ID` | Used by `IoSubsystem` for intake and launch (voltage control). |
| 12 | **Intake motor** | `IoConstants.INTAKE_MOTOR_ID` | Pulls fuel from floor or storage. **Anti-clockwise = intake; clockwise = spit out.** Defined in constants; not yet instantiated in `IoSubsystem` (for when hardware is wired). |
| 19 | Loader / directional wheel | `IoConstants.LOADER_MOTOR_ID` | Used by `IoSubsystem` for intake and launch (duty cycle 0–1). |

- `IoSubsystem` currently creates only **two** SPARK MAX controllers: **9** (IO motor) and **19** (loader). Both are **brushed**. Intake motor **12** is in `Constants` and in the printed CAN ID list; add it to `IoSubsystem` when the intake motor is on the bus.

---

## `Constants.java` reference

### `DriveConstants`

| Constant | Type | Meaning |
|----------|------|---------|
| `LEFT_LEADER_ID` | int | 11 – left leader drivetrain motor. |
| `LEFT_FOLLOWER_ID` | int | 8 – left follower. |
| `RIGHT_LEADER_ID` | int | 10 – right leader. |
| `RIGHT_FOLLOWER_ID` | int | 7 – right follower. |
| `DRIVE_MOTOR_CURRENT_LIMIT` | int | Smart current limit (amps) for drive motors. |
| `WHEEL_DIAMETER_METERS` | double | Wheel diameter for encoder distance. |
| `GEAR_RATIO` | double | Motor-to-wheel gear ratio. |

Used by `CANDriveSubsystem` and the `Drive` / `AutoDrive` commands.

### `IoConstants`

| Constant | Type | Meaning |
|----------|------|---------|
| `IO_MOTOR_ID` | int | 9 – IO / flywheel motor. |
| `INTAKE_MOTOR_ID` | int | 12 – intake motor (pulls fuel from floor/storage; anti-clockwise = intake, clockwise = spit out). |
| `LOADER_MOTOR_ID` | int | 19 – loader / directional wheel. |
| `IO_MOTOR_CURRENT_LIMIT` | int | Current limit (amps) for motor 9. |
| `LOADER_MOTOR_CURRENT_LIMIT` | int | Current limit (amps) for motor 19. |
| `INTAKING_IO_VOLTAGE` | double | IO motor voltage during intake. |
| `INTAKING_LOADER_OUTPUT` | double | Loader duty cycle (0–1) during intake. |
| `PREPARING_IO_VOLTAGE` / `PREPARING_LOADER_OUTPUT` | double | Optional “prepare” preset (not bound to a button). |
| `LAUNCHING_IO_VOLTAGE` | double | IO motor voltage during launch. |
| `LAUNCHING_LOADER_OUTPUT` | double | Loader duty cycle (0–1) during launch. |

### `OperatorConstants`

| Constant | Meaning |
|----------|---------|
| `DRIVER_CONTROLLER_PORT` | 0 – driver Xbox. |
| `OPERATOR_CONTROLLER_PORT` | 1 – reserved; not used in bindings. |
| `DRIVE_SCALING` | Multiplier on forward/back (e.g. 0.7). |
| `ROTATION_SCALING` | Multiplier on rotation (e.g. 0.8). |

### `Constants.getCanIdsList()`

Returns a formatted string of all CAN IDs (drivetrain + IO, intake, loader). Called from `Robot.robotInit()` and printed to console and SmartDashboard **"CAN IDs"**.

---

## Subsystems and commands

### `CANDriveSubsystem`

- Builds 4 brushed SPARK MAX controllers from `DriveConstants`.
- Voltage compensation (12 V), smart current limits, left-side inversion, followers.
- Public API: `driveArcade(xSpeed, zRotation)`, `driveTank(left, right)`, `driveVolts(left, right)`, `stop()`, encoder getters, `resetEncoders()`.
- **Default command:** `Drive(driveSubsystem, driverController)` — arcade drive from sticks.

### `IoSubsystem`

- Builds **two** brushed SPARK MAX: `IO_MOTOR_ID` (9), `LOADER_MOTOR_ID` (19). Intake (12) is in constants only.
- `setSpeeds(ioVoltage, loaderOutput)` — IO by voltage, loader by duty cycle.
- Commands: `commandIntake()`, `commandLaunch()`, `commandStop()`, `commandEject()`, `commandPrepare()`.
- **Default command:** `commandStop()` so fuel motors are off unless a command runs.
- **Bindings:** LT → `commandIntake()`, RT → `commandLaunch()`. B is on drive (reset encoders).

### Autonomous

- Chooser on SmartDashboard:
  - **Do Nothing** (default)
  - **Drive Forward 2s** — `AutoDrive(0.5, 0)` for 2 s
  - **(test) turn left 2s** — `AutoDrive(0, 0.1)` for 2 s
  - **(untested) Drive back & shoot preload left** — drive back, turn left, then `commandLaunch()` 1 s
  - **(untested) Drive back & shoot preload right** — drive back, turn right, then `commandLaunch()` 1 s

---

## Build & deploy

- **Build:** `./gradlew build` (or use the Gradle wrapper; ensure Java 17 and `JAVA_HOME` are set if needed).
- **Team number:** Set in WPILib VS Code or `.wpilib/wpilib_preferences.json`.
- **Deploy:** Use WPILib deploy; driver Xbox on **USB port 0**.
- Use the dashboard auto chooser to pick an autonomous before enabling.

Hello world!