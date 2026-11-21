package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;

/** Drives a straight Pedro path from wherever the robot is to a target pose. */
@Config
public class DriveToPoseCommand implements Command {

    public static double TIMEOUT_SEC = 5.0;

    private final Robot robot;
    private final Pose target;
    private final ElapsedTime timer = new ElapsedTime();

    /** Target in inches and degrees (field frame). */
    public DriveToPoseCommand(Robot robot, double xIn, double yIn, double headingDeg) {
        this.robot = robot;
        this.target = new Pose(xIn, yIn, Math.toRadians(headingDeg));
    }

    @Override
    public void initialize() {
        timer.reset();
        robot.drivetrain.followPath(Paths.line(robot.localization.getPose(), target));
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return !robot.drivetrain.isFollowingPath() || timer.seconds() >= TIMEOUT_SEC;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) robot.drivetrain.stop();
    }
}
