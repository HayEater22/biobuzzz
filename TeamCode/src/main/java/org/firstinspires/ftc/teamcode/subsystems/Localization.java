package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Field pose, read from the Pedro Pinpoint localizer.
 * Frame: +X forward, +Y left, heading CCW positive. Units: inches, radians (getters).
 *
 * The Pinpoint hardware is created and configured by the Pedro Follower (pedro/Constants), and
 * the Drivetrain's follower.update() reads it each loop. This subsystem is the single place the
 * rest of the robot gets and corrects the pose. The pose is not reset on init, so the pose at
 * the end of Autonomous carries into TeleOp.
 */
public class Localization implements Subsystem {

    private final Localizer localizer;

    public Localization(Drivetrain drivetrain) {
        this.localizer = drivetrain.getFollower().localizer;
    }

    /** Tell the Pinpoint exactly where the robot is (inches, degrees). */
    public void setPose(double xInches, double yInches, double headingDeg) {
        localizer.setPose(new Pose(xInches, yInches, Math.toRadians(headingDeg)));
    }

    public Pose getPose() { return localizer.pose(); }
    public double getX() { return localizer.pose().x(); }
    public double getY() { return localizer.pose().y(); }
    public double getHeading() { return localizer.pose().heading(); }

    @Override
    public void periodic(Telemetry telemetry) {
        telemetry.addData("Pose", "x=%.1f y=%.1f h=%.1f°", getX(), getY(), Math.toDegrees(getHeading()));
    }

    @Override
    public void stop() {}
}
