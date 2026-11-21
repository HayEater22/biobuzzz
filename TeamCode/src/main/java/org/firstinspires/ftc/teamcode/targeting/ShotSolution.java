package org.firstinspires.ftc.teamcode.targeting;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.util.MathUtil;

/** Turret angle, flywheel RPM and hood angle needed to hit the target goal from a given pose. */
@Config
public final class ShotSolution {

    /** Turret pivot offset from the robot center (inches, robot frame: +X forward, +Y left). */
    public static double TURRET_OFFSET_X = 0.0;
    public static double TURRET_OFFSET_Y = 0.0;

    public final double distanceIn;
    public final double turretAngleDeg;
    public final double rpm;
    public final double hood;

    private ShotSolution(double distanceIn, double turretAngleDeg, double rpm, double hood) {
        this.distanceIn = distanceIn;
        this.turretAngleDeg = turretAngleDeg;
        this.rpm = rpm;
        this.hood = hood;
    }

    /** Same aim, different flywheel RPM (manual RPM override in testing). */
    public ShotSolution withRpm(double newRpm) {
        return new ShotSolution(distanceIn, turretAngleDeg, newRpm, hood);
    }

    /**
     * @param x       robot x (in, field)
     * @param y       robot y (in, field)
     * @param heading robot heading (rad, CCW+)
     */
    public static ShotSolution compute(double x, double y, double heading, GoalSelector goal) {
        double cos = Math.cos(heading), sin = Math.sin(heading);
        double turretX = x + TURRET_OFFSET_X * cos - TURRET_OFFSET_Y * sin;
        double turretY = y + TURRET_OFFSET_X * sin + TURRET_OFFSET_Y * cos;

        double dx = goal.getGoalX() - turretX;
        double dy = goal.getGoalY() - turretY;
        double distance = Math.hypot(dx, dy);
        // Field bearing to the goal, minus robot heading = angle relative to robot forward.
        double turretAngle = Math.toDegrees(MathUtil.wrapRadians(Math.atan2(dy, dx) - heading));

        return new ShotSolution(distance, turretAngle, ShotTable.rpmFor(distance), ShotTable.hoodFor(distance));
    }
}
