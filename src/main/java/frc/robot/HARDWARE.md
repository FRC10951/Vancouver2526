# Hardware (this robot)

- Modified FRC **Rebuilt KitBot**
- **Seven Spark MAX:** four on **CIM** drivetrain (AndyMark-style brushed motors), three on **Neo** brushless motors for intake / loader / flywheel (internal encoders on the Neos; data ports available for extra sensors)
- **REV** PDP
- **roboRIO**

## Drive encoders

Brushed CIM motors do **not** include encoders. For wheel odometry, quadrature encoders must be wired to each **drive** Spark MAX **data port** and configured in firmware. This codebase uses `Constants.DriveConstants.DRIVE_QUADRATURE_ENCODERS_WIRED` — set it to `true` only when that wiring is in place and counts are valid. Until then, autonomous drive segments are **timed** open-loop (`AutoDrive` + timeouts), not distance-based.
