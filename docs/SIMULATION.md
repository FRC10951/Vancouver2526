# Running the Robot in Simulation

This project is configured to run the robot code in the **WPILib simulation environment** (desktop) with the **Simulation GUI** and **Driver Station**, so you can test logic without hardware.

> **Note:** This is the standard WPILib/Java simulation setup. If you were thinking of the legacy LabVIEW robot simulator, FRC Java projects use this WPILib simulator instead.

---

## Prerequisites

- **WPILib VS Code** (or Cursor with WPILib extension) with the project opened
- **Desktop support** is enabled in `build.gradle` (`includeDesktopSupport = true`)
- **Simulation** is enabled in `build.gradle` (`wpi.sim.addGui()`, `wpi.sim.addDriverstation()`)

---

## How to Run the Simulation

### Option 1: Simulate from the command palette (recommended)

1. Open the **Command Palette** (`Ctrl+Shift+P` / `Cmd+Shift+P`).
2. Run **WPILib: Simulate Robot Code**.
3. When prompted, choose **Sim GUI** (and optionally **Driver Station**) and press OK.
4. Wait for the build to finish. The **Simulation GUI** and **Driver Station** window will open.
5. In the **Driver Station** panel, set the robot to **Teleop** (or **Autonomous**) and click **Enable**.
6. Use your driver Xbox controller (USB port 0) or the Sim GUI keyboard joysticks. Connect **Shuffleboard** or **SmartDashboard** to **127.0.0.1** to see live NetworkTables data (see below).

### Option 2: Run from the Run and Debug view

1. Open the **Run and Debug** view (Ctrl+Shift+D).
2. Select **Simulate Robot Code** (or **WPILib Desktop Debug**) from the dropdown.
3. Press the green play button (F5).
4. Start the **Simulation GUI** and **Driver Station** if they are not started automatically.
5. Enable the robot in the Driver Station as above.

---

## FRC Dashboard (SmartDashboard / Shuffleboard)

1. Start simulation and **enable** the robot in the Driver Station.
2. Open **SmartDashboard** or **Shuffleboard** (**WPILib: Start Tool**).
3. Connect to **127.0.0.1** (localhost) for simulation.
4. **Organized keys** (prefixes group widgets in Shuffleboard / Sim GUI NetworkTables view):

| Prefix / key | Content |
|--------------|---------|
| **`Robot/`** | Battery (V), mode, simulation flag, brownout, DS/FMS attached, match time, alliance, station, sim hint, FPGA time (sim only), **Auto selected** string, **`Robot/Auto choices`** SendableChooser |
| **`Robot/Info/CAN IDs`** | Same text as legacy `CAN IDs` (duplicate for older layouts) |
| **`Drive/`** | Wheel odometry when encoders enabled; otherwise **`Drive/Odometry`** status string |
| **`IO/`** | Shooter/intake RPM telemetry when `IO_PID_TELEMETRY` is true |

Legacy key **`CAN IDs`** is still published for compatibility.

---

## What Is Simulated

- **Driver Station:** Enable/disable, mode (Teleop/Auto/Test), alliance/station when the FMS window supplies them — so you can exercise `getAutonomousCommand()` paths.
- **Command scheduler:** Teleop bindings, autonomous routines, and **SendableChooser** selection behave like on the roboRIO.
- **NetworkTables:** Same keys as on a real robot when the dashboard connects to localhost.
- **Spark / Neo REV physics:** This project does **not** currently attach a full `DifferentialDriveSim` / encoder sim to `CANDriveSubsystem`. Drivetrain **motor outputs** still run through the sim stack, but **distance/velocity odometry** is only meaningful when `DRIVE_QUADRATURE_ENCODERS_WIRED` is true on real hardware (or if you add explicit sim encoders later). Treat drive auto timing as **time-based** tuning, same as on the real bot without wheel encoders.

---

## Simulation config files in this repo

**Policy:** Files such as `simgui-ds.json` at the **project root** are **committed on purpose** so the team shares the same Simulation GUI / Driver Station layout (joystick indices, window positions, etc.).

- **`simgui.json`** — Sim GUI layout (NetworkTables / Plot windows). `Plot <0>` is enabled by default so you can graph e.g. `Robot/Battery (V)` after adding it to the plot.
- **`simgui-ds.json`** — Keyboard joystick mappings for sim.
- **`simgui-window.json`** — Window positions.

---

## Troubleshooting

| Issue | What to do |
|-------|------------|
| "Desktop support" or sim not found | Ensure `includeDesktopSupport = true` and `wpi.sim.addGui().defaultEnabled = true` in `build.gradle`, then reload the project. |
| Driver Station doesn’t enable | Make sure the **Simulation GUI** is running and the robot program has started. |
| Dashboard doesn’t show data | Connect to **127.0.0.1** and ensure the robot is **Enabled**. Add widgets for **`Robot/...`** keys. |
| Auto chooser not found | The chooser is published as **`Robot/Auto choices`** (not `Auto choices`). |
| Controller not responding | USB gamepad on port 0, or use **Joysticks** tab in Sim GUI. |

---

## Summary

1. Run **WPILib: Simulate Robot Code**.
2. Open **Sim GUI** + **Driver Station**; **Enable** teleop or auto.
3. Connect **Shuffleboard/SmartDashboard** to **127.0.0.1** and use the **`Robot/`**, **`Drive/`**, and **`IO/`** trees for tuning and debugging.
