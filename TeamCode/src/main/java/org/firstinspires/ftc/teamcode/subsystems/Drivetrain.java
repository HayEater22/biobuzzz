package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.MathUtil;

/**
 * Mecanum drive built on the Pedro Pathing Follower, which owns the drive motors and the
 * Pinpoint localizer (configured in pedro/Constants).
 *
 * TeleOp: field-oriented drive() sends manual powers to the Follower.
 * Auto:   followPath() hands control to Pedro.
 * periodic() calls follower.update() once per loop, which reads the Pinpoint AND drives the motors.
 *
 * The driver's "forward" is a separate heading offset. resetImu() re-zeros that offset instead of
 * overwriting the Pinpoint heading, so re-zeroing driving never breaks the field pose the turret aims with.
 */
@Config
public class Drivetrain implements Subsystem {

    public static double MAX_POWER = 1.0;

    private final Follower follower;

    /** Field heading (rad) that the driver considers "forward". */
    private double driverForwardHeading = 0.0;

    public Drivetrain(HardwareMap hardwareMap) {
        follower = Constants.create(hardwareMap);
    }

    /** The Pedro Follower (pose source for Localization, path following for Auto). */
    public Follower getFollower() { return follower; }

    /**
     * Driver field-oriented drive. Inputs in [-1, 1].
     * @param forward +away from the driver
     * @param left    +to the driver's left
     * @param turn    +rotate counter-clockwise
     */
    public void drive(double forward, double left, double turn) {
        double h = follower.pose().heading() - driverForwardHeading;
        double cos = Math.cos(h), sin = Math.sin(h);
        double robotForward = forward * cos + left * sin;
        double robotLeft    = -forward * sin + left * cos;
        follower.manual(robotForward * MAX_POWER, robotLeft * MAX_POWER, turn * MAX_POWER);
    }

    /** Re-zero field-oriented driving: whatever way the robot faces now becomes "forward". */
    public void resetImu() {
        driverForwardHeading = follower.pose().heading();
    }

    /** Set the field heading (degrees) that counts as "forward" for the driver. */
    public void setDriverForwardHeading(double headingDeg) {
        driverForwardHeading = Math.toRadians(headingDeg);
    }

    /** Start following a Pedro path. */
    public void followPath(Path path) {
        follower.follow(path);
    }

    /** True while a path is being followed. */
    public boolean isFollowingPath() {
        return follower.following();
    }

    @Override
    public void periodic(Telemetry telemetry) {
        follower.update();   // reads Pinpoint + applies drive output for the current mode
        telemetry.addData("Drive", "%s, driver heading %.1f°", follower.mode(),
                Math.toDegrees(MathUtil.wrapRadians(follower.pose().heading() - driverForwardHeading)));
    }

    @Override
    public void stop() {
        follower.stop();
    }
}
