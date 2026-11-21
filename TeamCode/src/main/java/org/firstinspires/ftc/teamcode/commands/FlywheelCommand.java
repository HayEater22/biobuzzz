package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.targeting.ShotSolution;

/**
 * Keeps the flywheel at the right speed for the current distance to the goal, until cancelled.
 * Ending it idles the flywheel.
 */
public class FlywheelCommand implements Command {

    private final Robot robot;
    private final TrackGoalCommand tracker;

    public FlywheelCommand(Robot robot, TrackGoalCommand tracker) {
        this.robot = robot;
        this.tracker = tracker;
    }

    @Override
    public void execute() {
        ShotSolution solution = tracker.getSolution();
        if (solution != null) robot.shooter.setTargetRpm(solution.rpm);
    }

    @Override
    public boolean isFinished() { return false; }

    @Override
    public void end(boolean interrupted) { robot.shooter.idle(); }
}
