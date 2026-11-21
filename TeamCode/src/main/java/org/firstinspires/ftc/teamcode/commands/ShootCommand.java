package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.targeting.ShotSolution;

/**
 * Shooting sequence:
 *   1. SPIN_UP     gate closed; waits until the flywheel is at speed and the turret/hood are aimed.
 *   2. FEEDING     gate opens, intake + transfer run full forward to push every piece through.
 * Ending (trigger released, or feed time over) closes the gate and stops the motors. The flywheel is
 * idled too, unless this is the TeleOp version, where {@link FlywheelCommand} keeps it spinning.
 */
public class ShootCommand implements Command {

    public enum Stage { SPIN_UP, FEEDING, DONE }

    private final Robot robot;
    private final TrackGoalCommand tracker;
    private final double readyTimeoutSec;
    private final double feedSec;
    private final boolean idleFlywheelOnEnd;
    private final ElapsedTime timer = new ElapsedTime();
    private Stage stage = Stage.DONE;

    /**
     * @param readyTimeoutSec feed anyway after this long in SPIN_UP (infinite = wait until ready)
     * @param feedSec         finish after feeding this long (infinite = until cancelled)
     */
    public ShootCommand(Robot robot, TrackGoalCommand tracker, double readyTimeoutSec, double feedSec) {
        this(robot, tracker, readyTimeoutSec, feedSec, true);
    }

    private ShootCommand(Robot robot, TrackGoalCommand tracker, double readyTimeoutSec, double feedSec,
                         boolean idleFlywheelOnEnd) {
        this.robot = robot;
        this.tracker = tracker;
        this.readyTimeoutSec = readyTimeoutSec;
        this.feedSec = feedSec;
        this.idleFlywheelOnEnd = idleFlywheelOnEnd;
    }

    /**
     * TeleOp version: runs while the button is held, only feeds when actually ready,
     * and leaves the flywheel spinning afterwards (run alongside {@link FlywheelCommand}).
     */
    public static ShootCommand whileHeld(Robot robot, TrackGoalCommand tracker) {
        return new ShootCommand(robot, tracker, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, false);
    }

    /** Test version: like whileHeld, but feeds immediately without waiting for RPM or aim. */
    public static ShootCommand whileHeldNoWait(Robot robot, TrackGoalCommand tracker) {
        return new ShootCommand(robot, tracker, 0.0, Double.POSITIVE_INFINITY, false);
    }

    @Override
    public void initialize() {
        timer.reset();
        robot.transfer.closeGate();
        robot.intake.stop();
        robot.transfer.stopMotor();
        stage = Stage.SPIN_UP;
    }

    @Override
    public void execute() {
        ShotSolution solution = tracker.getSolution();
        if (solution != null) robot.shooter.setTargetRpm(solution.rpm);

        switch (stage) {
            case SPIN_UP:
                boolean ready = robot.shooter.isAtSpeed() && tracker.isAimed();
                if (ready || timer.seconds() >= readyTimeoutSec) {
                    robot.transfer.openGate();
                    robot.intake.intake();
                    robot.transfer.feed();
                    stage = Stage.FEEDING;
                    timer.reset();
                }
                break;
            case FEEDING:
                // Keep feeding even as RPM dips from each shot.
                if (timer.seconds() >= feedSec) stage = Stage.DONE;
                break;
            default:
                break;
        }
    }

    public Stage getStage() { return stage; }

    @Override
    public boolean isFinished() { return stage == Stage.DONE; }

    @Override
    public void end(boolean interrupted) {
        stage = Stage.DONE;
        robot.transfer.closeGate();
        robot.transfer.stopMotor();
        robot.intake.stop();
        if (idleFlywheelOnEnd) robot.shooter.idle();
    }
}
