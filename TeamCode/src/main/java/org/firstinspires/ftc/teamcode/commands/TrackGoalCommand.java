package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.targeting.ShotSolution;

/**
 * Every loop: computes the shot solution from the Pinpoint pose and points the turret and hood
 * at the selected goal. Runs for the whole OpMode. The flywheel is left to {@link FlywheelCommand}
 * and {@link ShootCommand}.
 */
public class TrackGoalCommand implements Command {

    private final Robot robot;
    private ShotSolution solution;
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

    @Override
    public void execute() {
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
