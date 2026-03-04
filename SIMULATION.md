# Running the Robot in Simulation

This project is configured to run the robot code in the **WPILib simulation environment** (desktop) with the **Simulation GUI** and **Driver Station**, so you can test your code without hardware.

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
6. Use your driver Xbox controller (USB port 0) to drive. SmartDashboard/Shuffleboard will show live data when connected (see below).

### Option 2: Run from the Run and Debug view

1. Open the **Run and Debug** view (Ctrl+Shift+D).
2. Select **Simulate Robot Code** (or **WPILib Desktop Debug**) from the dropdown.
3. Press the green play button (F5).
4. Start the **Simulation GUI** and **Driver Station** if they are not started automatically (via WPILib commands or from the Sim GUI launcher).
5. Enable the robot in the Driver Station as above.

---

## FRC Dashboard (SmartDashboard / Shuffleboard)

To view live data from the simulated robot:

1. **Start the simulation** using one of the options above and **enable** the robot in the Driver Station.
2. Open **SmartDashboard** or **Shuffleboard** from the WPILib extension (e.g. **WPILib: Start Tool** → SmartDashboard or Shuffleboard).
3. **Connect to the simulator:**
   - Set the **team number** to **127.0.0.1** (localhost), or enter your team number and ensure the tool is set to connect to the simulation host.
   - SmartDashboard/Shuffleboard will connect over NetworkTables to the running sim.
4. You should see:
   - **CAN IDs** (from `Constants.getCanIdsList()`)
   - **Left/Right Distance (m)** and **Left/Right Velocity (m/s)** from the drivetrain
   - **Autonomous chooser** options

---

## What Is Simulated

- **Drivetrain:** `DifferentialDrivetrainSim` (KitBot: dual CIM per side, 10.71:1, 6" wheels). Encoder positions and velocities are updated from the sim so autonomous and teleop drive logic behave as on a real robot.
- **Driver Station:** Enable/disable, mode (Teleop/Auto/Test), so you can test teleop and auto.
- **SmartDashboard/Shuffleboard:** Same NetworkTables data as on the robot when connected to localhost.

---

## Troubleshooting

| Issue | What to do |
|-------|------------|
| "Desktop support" or sim not found | Ensure `includeDesktopSupport = true` and `wpi.sim.addGui().defaultEnabled = true` in `build.gradle`, then reload the project. |
| Driver Station doesn’t enable | Make sure the **Simulation GUI** is running and the robot program has started (e.g. "Simulate Robot Code" or "WPILib Desktop Debug"). |
| Dashboard doesn’t show data | Connect to **127.0.0.1** (or the correct sim host) in SmartDashboard/Shuffleboard and ensure the robot is **Enabled** in the Driver Station. |
| Controller not responding | Plug the Xbox controller into **USB port 0** (driver port). Check the Sim GUI’s **Joysticks** tab to confirm it’s detected. |

---

## Summary

1. Run **WPILib: Simulate Robot Code** (or **Simulate Robot Code** from Run and Debug).
2. Open the **Simulation GUI** and **Driver Station** (if not automatic).
3. **Enable** the robot in the Driver Station (Teleop or Autonomous).
4. Open **SmartDashboard** or **Shuffleboard** and connect to **127.0.0.1** to view the FRC dashboard with live data from your code.
