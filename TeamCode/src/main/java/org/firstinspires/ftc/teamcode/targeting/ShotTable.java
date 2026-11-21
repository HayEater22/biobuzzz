package org.firstinspires.ftc.teamcode.targeting;

import com.acmerobotics.dashboard.config.Config;
/**
 * Lookup table: distance to goal -> flywheel RPM and hood angle, linearly interpolated.
 *
 * How to fill it in: place the robot at a measured distance (the TeleOp shows "Distance"),
 * adjust RPM and HOOD until shots go in reliably, and write down the row. Repeat across
 * the shooting range. More rows = more accurate. Keep DISTANCE_IN ascending.
 */
@Config
public final class ShotTable {
    private ShotTable() {}

    // ---- Tuning (placeholder values) ----
    public static double[] DISTANCE_IN = {24, 48, 72, 96, 120, 144};
    public static double[] RPM         = {2400, 2700, 3000, 3300, 3600, 3900};
    public static double[] HOOD        = {0.10, 0.25, 0.40, 0.55, 0.70, 0.80};

    // ---- Tuning helper ----
    /** true = ignore the table and use OVERRIDE_RPM / OVERRIDE_HOOD. Use while finding each row on the Dashboard. */
    public static boolean OVERRIDE = false;
    public static double OVERRIDE_RPM = 3000;
    public static double OVERRIDE_HOOD = 0.5;

    public static double rpmFor(double distanceIn)  { return OVERRIDE ? OVERRIDE_RPM  : interpolate(DISTANCE_IN, RPM, distanceIn); }
    public static double hoodFor(double distanceIn) { return OVERRIDE ? OVERRIDE_HOOD : interpolate(DISTANCE_IN, HOOD, distanceIn); }

    /** Piecewise-linear lookup; clamps outside the table. */
    static double interpolate(double[] xs, double[] ys, double x) {
        if (x <= xs[0]) return ys[0];
        int last = xs.length - 1;
        if (x >= xs[last]) return ys[last];
        for (int i = 0; i < last; i++) {
            if (x <= xs[i + 1]) {
                double t = (x - xs[i]) / (xs[i + 1] - xs[i]);
                return ys[i] + t * (ys[i + 1] - ys[i]);
            }
        }
        return ys[last];
    }
}
