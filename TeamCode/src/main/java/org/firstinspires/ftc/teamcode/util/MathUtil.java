package org.firstinspires.ftc.teamcode.util;

public final class MathUtil {
    private MathUtil() {}

    /** Wrap an angle to (-pi, pi]. */
    public static double wrapRadians(double radians) {
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians <= -Math.PI) radians += 2 * Math.PI;
        return radians;
    }

    /** Move current toward target by at most maxStep. */
    public static double approach(double current, double target, double maxStep) {
        double delta = target - current;
        if (Math.abs(delta) <= maxStep) return target;
        return current + Math.signum(delta) * maxStep;
    }
}
