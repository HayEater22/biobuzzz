package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;

/**
 * BIOBUZZ field geometry in the Pinpoint / Pedro field frame (inches, 0..144).
 *
 *   (0,0)     = the field corner inside the RED GARDEN (on the audience wall, red side)
 *   +X        = along the audience wall, from the RED side wall to the BLUE side wall
 *   +Y        = away from the audience, toward the rear wall
 *   (144,144) = the corner inside the BLUE GARDEN
 *   Heading   = degrees CCW, 0 deg points along +X
 *
 * From the 2026-27 Competition Manual:
 *   - Red ALLIANCE AREA is on the left wall as seen from the audience (X = 0), blue on the right (X = 144).
 *   - The HIVE frame stands in the center of the field: two triangular metal frames ~49.46 in apart
 *     (so at about X = 47.3 and X = 96.7), joined by a crossbar with the pivots 43.95 in up.
 *     Red HIVE hangs on the red half (X < 72), blue HIVE on the blue half.
 *   - Each HIVE has a CELL on its audience end and its rear end and tips between them
 *     (measured: upward cell sits 48 in from the audience or rear wall).
 *     AprilTags on the CELL undersides: red rear 30-33, red audience 34-37, blue audience 38-41, blue rear 42-45.
 */
@Config
public final class FieldConstants {
    private FieldConstants() {}

    public static final double FIELD_SIZE_IN = 144.0;

    public enum Alliance { RED, BLUE }

    /** Which end of the HIVE the upward (scorable) cell is on. Swaps each time the hive tips. */
    public enum HiveEnd { AUDIENCE, REAR }

    // ---- HIVE cells: aim point = center of the upward CELL opening (X/Y only) ----
    // Measured on the field (floor spot under the opening): 4 ft from the audience / rear wall,
    // ~57 in from the cell's own alliance wall (blue audience cell 57.5 in).
    public static double RED_HIVE_AUDIENCE_X  = 57.0;
    public static double RED_HIVE_AUDIENCE_Y  = 48.0;
    public static double RED_HIVE_REAR_X      = 57.0;
    public static double RED_HIVE_REAR_Y      = 96.0;
    public static double BLUE_HIVE_AUDIENCE_X = FIELD_SIZE_IN - 57.5;   // 86.5
    public static double BLUE_HIVE_AUDIENCE_Y = 48.0;
    public static double BLUE_HIVE_REAR_X     = FIELD_SIZE_IN - 57.0;   // 87.0
    public static double BLUE_HIVE_REAR_Y     = 96.0;

    // ---- Automatic goal switching ----
    /** true = aim at the AUDIENCE cell while the robot is on the audience half, REAR cell on the rear half. */
    public static boolean AUTO_SWITCH_HIVE_END = true;
    /** The audience/rear center line. */
    public static double CENTER_LINE_Y = 72.0;
    /** Must cross this far past the line before switching, so the turret doesn't flip back and forth on it. */
    public static double CENTER_LINE_HYSTERESIS_IN = 2.0;

    // ---- Zero / relocalize spot (triple-tap X) ---- TUNE
    // Front bumper rammed against the HIVE frame's metal leg at TILE seam (3, W) for red / (3, Y) for blue
    // (manual Figure 9-4), robot facing straight away from its own driver station, intake over the beam.
    /** Seam 3 = center line, Y = 72. */
    public static double ZERO_SEAM_Y = 72.0;
    /** Seam W: the frame leg the RED robot's front bumper touches. */
    public static double RED_ZERO_BEAM_X = 48.0;
    /** Seam Y: the frame leg the BLUE robot's front bumper touches. */
    public static double BLUE_ZERO_BEAM_X = 96.0;
    /** Robot center to the beam it is touching (176.316 mm). */
    public static double ZERO_FRONT_TO_CENTER_IN = 6.942;

    // ---- Driver "forward" for field-oriented drive ----
    /** Field heading pointing straight away from that alliance's driver station (red wall X=0, blue wall X=144). */
    public static double RED_DRIVER_HEADING_DEG  = 0.0;
    public static double BLUE_DRIVER_HEADING_DEG = 180.0;

    // ---- Hive ----
    public static double hiveCellX(Alliance a, HiveEnd end) {
        if (a == Alliance.RED) return end == HiveEnd.AUDIENCE ? RED_HIVE_AUDIENCE_X : RED_HIVE_REAR_X;
        return end == HiveEnd.AUDIENCE ? BLUE_HIVE_AUDIENCE_X : BLUE_HIVE_REAR_X;
    }

    public static double hiveCellY(Alliance a, HiveEnd end) {
        if (a == Alliance.RED) return end == HiveEnd.AUDIENCE ? RED_HIVE_AUDIENCE_Y : RED_HIVE_REAR_Y;
        return end == HiveEnd.AUDIENCE ? BLUE_HIVE_AUDIENCE_Y : BLUE_HIVE_REAR_Y;
    }

    /**
     * Upward cell at the start of a match. Manual: the CELL that points at a FLOWER starts tilted DOWN.
     * TUNE once you know which ends those are (D-pad up/down in TeleOp also sets it).
     */
    public static HiveEnd startingHiveEnd(Alliance a) {
        return a == Alliance.RED ? HiveEnd.AUDIENCE : HiveEnd.REAR;
    }

    // ---- Driver heading ----
    public static double driverHeadingDeg(Alliance a) {
        return a == Alliance.BLUE ? BLUE_DRIVER_HEADING_DEG : RED_DRIVER_HEADING_DEG;
    }

    // ---- Zero / relocalize (triple-tap X) ----
    // Red faces +X, so its center sits on the red-wall side (lower X) of the W leg; blue faces -X, center past the Y leg.
    public static double relocalizeX(Alliance a) {
        return a == Alliance.RED ? RED_ZERO_BEAM_X - ZERO_FRONT_TO_CENTER_IN
                                 : BLUE_ZERO_BEAM_X + ZERO_FRONT_TO_CENTER_IN;
    }

    public static double relocalizeY(Alliance a) {
        return ZERO_SEAM_Y;
    }

    /** Facing forward from the driver's view, intake over the beam. */
    public static double relocalizeHeadingDeg(Alliance a) {
        return driverHeadingDeg(a);
    }
}
