# FTC Command-Based Architecture (`CLAUDE.md`)

## Core Objective
Structure and build this FTC Java codebase (`org.firstinspires.ftc.teamcode`) into a strict **command-based / subsystem-based architecture**.

- **Subsystems (`subsystems/`):** Dedicated classes for each mechanism. Subsystems own all `hardwareMap` calls, motor/servo control, sensor reads, PID loops, and safety limits.
- **Commands / Actions (`commands/`):** Reusable action classes or methods that coordinate multiple subsystems (e.g., aiming, spinning up, indexing, adjusting hood).
- **OpModes (`opmodes/teleop/` & `opmodes/auto/`):** Must contain **zero raw hardware calls or low-level motor/servo math**. OpModes only instantiate subsystems and call clean, high-level functions.

---

## Required Subsystems & Architecture
1. **Drivetrain Subsystem:** Handles movement and localization (including goBILDA Pinpoint V2 odometry and pose tracking).
2. **Intake Subsystem:** Manages intake mechanisms and game piece collection.
3. **Transfer Subsystem:** Manages indexing and moving game pieces from intake to shooter.
4. **Turret Subsystem:** Dedicated position control for the rotating turret (with software soft-limits to protect wiring).
5. **Shooter Subsystem:** Dedicated velocity control for the flywheel (RPM/PIDF).
6. **Hood Subsystem:** Dedicated angle/position control for the shooter hood (adjustable based on distance/targeting calculations).

---

## Architecture & Coding Rules
1. **Discover Hardware First:** Scan the repository and `build.gradle` to identify the exact motors, servos, sensors, and libraries actually present. Never invent hardware devices that aren't in the repo.
2. **Subsystem Contract:**
   - Every subsystem initializes its own hardware in its constructor via `HardwareMap`.
   - Expose clear, high-level action methods and boolean state checks (e.g., `isAtTarget()`, `isAtSpeed()`).
   - Include a `periodic()` method called once per loop iteration to update control loops and telemetry.
3. **Dynamic Targeting:** Allow subsystems (like turret angle, shooter velocity, and hood position) to accept dynamic target inputs computed from Pinpoint odometry pose data.
4. **Loop Efficiency:** Enable `LynxModule.BulkCachingMode.MANUAL` and clear the bulk cache once at the top of every OpMode loop.
5. **Live Tuning:** Expose PID constants, target speeds, and positions as `public static` fields for easy tuning.
6. **Verify Compilation:** Run `./gradlew assembleDebug` after making changes to confirm the project builds with zero errors.

---

## Immediate First Step
Scan `build.gradle` and all existing Java files in `TeamCode` to list every hardware device and library currently used, then scaffold the `subsystems/`, `commands/`, and `opmodes/` packages around your drive, intake, transfer, turret, shooter, and hood subsystems.
