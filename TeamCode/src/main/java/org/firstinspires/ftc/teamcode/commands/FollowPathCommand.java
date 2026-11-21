package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * Follows any Pedro path (build one with com.pedropathing.api.Paths: line, curve, through).
 * Finishes when the follower is done with the path.
 */
public class FollowPathCommand implements Command {

    private final Robot robot;
    private final Path path;

    public FollowPathCommand(Robot robot, Path path) {
        this.robot = robot;
        this.path = path;
    }

    @Override
    public void initialize() { robot.drivetrain.followPath(path); }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() { return !robot.drivetrain.isFollowingPath(); }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) robot.drivetrain.stop();
    }
}
