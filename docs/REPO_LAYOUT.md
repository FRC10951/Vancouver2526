# Repository layout

Quick map of this KitBot project so new contributors know where things live.

| Path | Purpose |
|------|---------|
| `src/main/java/frc/robot/` | Robot application code (`Robot`, `RobotContainer`, `Constants`, commands, subsystems). |
| `src/main/deploy/` | Files deployed to the roboRIO under `/home/lvuser/deploy` (e.g. config assets). |
| `vendordeps/` | Third-party JSON descriptors (WPILib New Commands, REVLib, etc.). |
| `gradle/`, `gradlew`, `gradlew.bat`, `build.gradle`, `settings.gradle` | Gradle build and WPILib (GradleRIO) configuration. |
| `.wpilib/` | WPILib team preferences (e.g. team number). |
| `.vscode/` | Shared VS Code / Cursor launch and task hints (optional for the team). |
| `.cursor/` | Cursor-specific agent hints (optional; safe to keep for teams using Cursor). |
| `tools/constants-editor/` | Swing GUI subproject to edit `Constants.java` (run via Gradle or `tools/RunConstantsEditor.bat`). |
| `tools/RunConstantsEditor.bat` | Launches the constants editor from the repo root. |
| `scripts/` | Non-robot scripts (e.g. PowerShell helpers). |
| `docs/` | Longer documentation (`SIMULATION.md`, debugging notes, this file). |
| `constantsArchive/` | **Not tracked in git** (see `.gitignore`). Previous `Constants.java` revisions saved by the constants editor as numbered `.txt` files. |
| `README.md` | Main project overview: hardware, controls, constants, build/deploy. |
| `simgui-ds.json` (root) | Simulation Driver Station layout; **committed** so the team can share defaults (see `docs/SIMULATION.md`). |

## Java package structure

- `frc.robot.commands` — Command classes (teleop drive, auto routines, etc.).
- `frc.robot.subsystems` — Subsystems (`CANDriveSubsystem`, `IoSubsystem`, …).

## Related reading

- [SIMULATION.md](SIMULATION.md) — Running desktop simulation, Sim GUI layout, and **`Robot/` / `Drive/` / `IO/`** dashboard keys.
- [IO_PID.md](IO_PID.md) — Shooter and intake velocity PID + feedforward tuning.
- [README.md](../README.md) — Robot behavior, CAN map, and operator controls.
