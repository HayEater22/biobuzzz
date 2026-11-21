package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;

/**
 * Settings for "TeleOp (Turret Locked)" (runs as the Blue alliance).
 * No X/Y odometry is used; the shot uses a fixed RPM and hood. Only the gyro heading is used, for the
 * D-pad left/right field headings. Edit here, or live on the FTC Dashboard under "LockedShotConfig".
 */
@Config
public final class LockedShotConfig {
    private LockedShotConfig() {}

    /**
     * X for the turret's FIELD headings (degrees; 0 = from the red wall toward the blue wall, CCW +).
     * The turret holds the chosen field heading while the robot turns:
     *   Red:  D-pad left = 180 + X,  D-pad right = 360 - X
     *   Blue: D-pad left = 180 - X,  D-pad right = X        (Red mirrored across the field)
     * D-pad up or down = turret straight forward on the robot (0°).
     * Heading must be set first: run Auto before, or triple-tap X at the beam.
     */
    public static double TURRET_OFFSET_DEG = 23;

    /**
     * Correction added to every field heading above (degrees, + = turret turns counter-clockwise / left).
     * Set to 90 because the turret pointed 90° clockwise of the intended direction.
     * If it's now 180° off, use -90 instead.
     */
    public static double FIELD_HEADING_CORRECTION_DEG = 90.0;

    /**
     * true = when the robot crosses the audience/rear center line (field Y = 72), the D-pad left/right
     * choice swaps sides automatically (like the normal TeleOp switching goal ends).
     * Uses odometry Y position, so relocalize (triple-tap X) at the start.
     */
    public static boolean SWAP_SIDES_AT_CENTER_LINE = true;

    /** Flywheel RPM (3000 + 5%). */
    public static double RPM = 3250;

    /** Hood position (0..1). */
    public static double HOOD = 0.68;

    /**
     * If the flywheel isn't at speed this long after the shoot hold starts, feed anyway.
     * Keep it LONGER than spin-up from a stopped wheel: RPM / Shooter.RAMP_RPM_PER_SEC
     * (3150 / 2000 = about 1.6 s), or the first ball can leave slow.
     * Read when the OpMode is initialized (re-init after changing it on the Dashboard).
     */
    public static double READY_TIMEOUT_SEC = 2.0;

    /**
     * One ball at a time: after each shot the feed pauses until the flywheel is back within
     * Shooter.RPM_TOLERANCE of RPM, or this long at most (so it never stalls).
     */
    public static double MAX_RECOVERY_WAIT_SEC = 0.4;

    /** RPM drop (from the highest RPM since the feed last started) that counts as "a ball just went through". */
    public static double SHOT_DIP_RPM = 100;
}
