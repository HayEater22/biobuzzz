package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.config.FieldConstants;
import org.firstinspires.ftc.teamcode.config.LockedShotConfig;
import org.firstinspires.ftc.teamcode.targeting.ShotSolution;

/**
 * Every loop: computes the shot solution from the Pinpoint pose and points the turret and hood
 * at the selected goal. Runs for the whole OpMode. The flywheel is left to {@link FlywheelCommand}
 * and {@link ShootCommand}.
 */
public class TrackGoalCommand implements Command {

    private final Robot robot;
    private ShotSolution solution;
    /** true = ignore the pose: fixed turret angle, RPM and hood from {@link LockedShotConfig}. */
    private boolean fixedShot = false;
    /** Fixed-shot turret: 0 = robot forward, +1 = D-pad left field heading, -1 = D-pad right field heading. */
    private int lockedOffsetSign = 0;
    /** Manual flywheel RPM (testing). NaN = use the shot table. */
    private double rpmOverride = Double.NaN;

    public TrackGoalCommand(Robot robot) {
        this.robot = robot;
    }

    /** Use this RPM instead of the shot table's (aim and hood still track). */
    public void setRpmOverride(double rpm) { rpmOverride = Math.max(0.0, rpm); }
    /** Back to the shot table's RPM. */
    public void clearRpmOverride() { rpmOverride = Double.NaN; }
    public boolean hasRpmOverride() { return !Double.isNaN(rpmOverride); }

    /** Fixed-shot mode: turret at a fixed angle, fixed RPM and hood, odometry ignored. */
    public void setFixedShot(boolean on) { fixedShot = on; }
    public boolean isFixedShot() { return fixedShot; }

    /** Fixed-shot RPM / hood for this OpMode only (shot sample OpModes). NaN = use LockedShotConfig. */
    private double fixedRpmOverride = Double.NaN;
    private double fixedHoodOverride = Double.NaN;

    public void setFixedShotValues(double rpm, double hood) {
        fixedRpmOverride = rpm;
        fixedHoodOverride = hood;
    }

    public double getFixedRpm()  { return Double.isNaN(fixedRpmOverride)  ? LockedShotConfig.RPM  : fixedRpmOverride; }
    public double getFixedHood() { return Double.isNaN(fixedHoodOverride) ? LockedShotConfig.HOOD : fixedHoodOverride; }

    /** Fixed-shot turret position: +1 = +offset (left), -1 = -offset (right), 0 = center. */
    public void setLockedOffset(int sign) {
        lockedOffsetSign = Integer.signum(sign);
    }

    /** True when D-pad left/right picked a field heading (false = turret straight forward on the robot). */
    public boolean isLockedFieldHeading() { return lockedOffsetSign != 0; }

    /**
     * Field heading (degrees, 0 = from the red wall toward the blue wall, CCW +) the turret holds for
     * D-pad left/right, with X = TURRET_OFFSET_DEG:  Red: 180+X / 360-X,  Blue: 180-X / X.
     * One alliance is the other mirrored across the field's center line, so each D-pad button aims at
     * the same side of the field for both alliances.
     */
    public double getLockedFieldHeadingDeg() {
        double x = LockedShotConfig.TURRET_OFFSET_DEG;
        boolean red = robot.getAlliance() == FieldConstants.Alliance.RED;
        if (red) return lockedOffsetSign > 0 ? 180.0 + x : 360.0 - x;
        return lockedOffsetSign > 0 ? 180.0 - x : x;
    }

    /**
     * Fixed-shot turret angle right now (degrees, robot frame, + = left). D-pad up/down: 0 (forward).
     * D-pad left/right: holds a FIELD heading, so the turret counter-rotates as the robot turns
     * (uses only the gyro heading, not X/Y position).
     */
    public double getLockedTurretAngle() {
        if (lockedOffsetSign == 0) return 0.0;
        double robotDeg = Math.toDegrees(robot.localization.getHeading());
        double a = getLockedFieldHeadingDeg() + LockedShotConfig.FIELD_HEADING_CORRECTION_DEG - robotDeg;
        return ((a + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;   // wrap to -180..180
    }

    @Override
    public void execute() {
        if (fixedShot) {
            solution = ShotSolution.fixed(getLockedTurretAngle(), getFixedRpm(), getFixedHood());
            robot.turret.setAngle(solution.turretAngleDeg);
            robot.hood.setAngle(solution.hood);
            return;
        }
        robot.goal.updateFromPose(robot.localization.getY());   // audience / rear half picks the goal
        solution = ShotSolution.compute(robot.localization.getX(), robot.localization.getY(),
                robot.localization.getHeading(), robot.goal);
        if (hasRpmOverride()) solution = solution.withRpm(rpmOverride);
        robot.turret.setAngle(solution.turretAngleDeg);
        robot.hood.setAngle(solution.hood);
    }

    /** Latest solution, or null before the first loop. */
    public ShotSolution getSolution() { return solution; }

    /** Turret and hood are on target, and the goal is within the turret's soft limits. */
    public boolean isAimed() {
        return solution != null
                && robot.turret.isAtTarget()
                && !robot.turret.isAtLimit()
                && robot.hood.isAtTarget();
    }

    @Override
    public boolean isFinished() { return false; }

    @Override
    public void end(boolean interrupted) { robot.turret.center(); }
}
