package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.config.LockedShotConfig;
import org.firstinspires.ftc.teamcode.targeting.ShotSolution;

/**
 * Shooting sequence:
 *   1. SPIN_UP     gate closed; waits until the flywheel is at speed and the turret/hood are aimed.
 *   2. FEEDING     gate opens, intake + transfer run full forward to push every piece through.
 *                  With "one ball at a time" on (Turret Locked TeleOp), the feed PAUSES after each shot
 *                  (the RPM drops by LockedShotConfig.SHOT_DIP_RPM) until the flywheel is back at speed,
 *                  or LockedShotConfig.MAX_RECOVERY_WAIT_SEC has passed, then feeds the next ball.
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
    /** Pause the feed after each shot until the flywheel recovers. */
    private final boolean oneBallAtATime;
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime pauseTimer = new ElapsedTime();
    private Stage stage = Stage.DONE;
    /** FEEDING: true = intake + transfer running, false = paused, waiting for the flywheel to recover. */
    private boolean feedRunning = false;
    /** Highest flywheel RPM since the feed last (re)started. A drop from here = a ball just went through. */
    private double peakRpm = 0.0;

    /**
     * @param readyTimeoutSec feed anyway after this long in SPIN_UP (infinite = wait until ready)
     * @param feedSec         finish after feeding this long (infinite = until cancelled)
     */
    public ShootCommand(Robot robot, TrackGoalCommand tracker, double readyTimeoutSec, double feedSec) {
        this(robot, tracker, readyTimeoutSec, feedSec, true, false);
    }

    private ShootCommand(Robot robot, TrackGoalCommand tracker, double readyTimeoutSec, double feedSec,
                         boolean idleFlywheelOnEnd, boolean oneBallAtATime) {
        this.robot = robot;
        this.tracker = tracker;
        this.readyTimeoutSec = readyTimeoutSec;
        this.feedSec = feedSec;
        this.idleFlywheelOnEnd = idleFlywheelOnEnd;
        this.oneBallAtATime = oneBallAtATime;
    }

    /**
     * TeleOp version: runs while the button is held, only feeds when actually ready,
     * and leaves the flywheel spinning afterwards (run alongside {@link FlywheelCommand}).
     */
    public static ShootCommand whileHeld(Robot robot, TrackGoalCommand tracker) {
        return new ShootCommand(robot, tracker, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, false, false);
    }

    /**
     * Like whileHeld, but if the flywheel hasn't reached speed (and aim) within readyTimeoutSec,
     * it starts feeding anyway.
     * @param oneBallAtATime pause the feed after each shot until the flywheel recovers
     */
    public static ShootCommand whileHeldWithTimeout(Robot robot, TrackGoalCommand tracker, double readyTimeoutSec,
                                                    boolean oneBallAtATime) {
        return new ShootCommand(robot, tracker, readyTimeoutSec, Double.POSITIVE_INFINITY, false, oneBallAtATime);
    }

    /** Test version: like whileHeld, but feeds immediately without waiting for RPM or aim. */
    public static ShootCommand whileHeldNoWait(Robot robot, TrackGoalCommand tracker) {
        return new ShootCommand(robot, tracker, 0.0, Double.POSITIVE_INFINITY, false, false);
    }

    @Override
    public void initialize() {
        timer.reset();
        robot.transfer.closeGate();
        setFeed(false);
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
                    setFeed(true);
                    stage = Stage.FEEDING;
                    timer.reset();
                }
                break;
            case FEEDING:
                if (oneBallAtATime) paceFeed();
                if (timer.seconds() >= feedSec) stage = Stage.DONE;
                break;
            default:
                break;
        }
    }

    /**
     * One ball at a time: when the RPM drops (a ball just went through), pause the feed; resume once the
     * flywheel is back within tolerance of its target, or after MAX_RECOVERY_WAIT_SEC so it never stalls.
     */
    private void paceFeed() {
        double rpm = robot.shooter.getRpm();
        if (feedRunning) {
            peakRpm = Math.max(peakRpm, rpm);
            if (peakRpm - rpm >= LockedShotConfig.SHOT_DIP_RPM) {
                setFeed(false);
                pauseTimer.reset();
            }
        } else if (robot.shooter.isAtSpeed() || pauseTimer.seconds() >= LockedShotConfig.MAX_RECOVERY_WAIT_SEC) {
            setFeed(true);
        }
    }

    /** Run or stop the intake + transfer (the gate is handled separately). */
    private void setFeed(boolean on) {
        feedRunning = on;
        if (on) {
            robot.intake.intake();
            robot.transfer.feed();
            peakRpm = robot.shooter.getRpm();
        } else {
            robot.intake.stop();
            robot.transfer.stopMotor();
        }
    }

    public Stage getStage() { return stage; }

    /** True while one-ball-at-a-time feeding is paused, waiting for the flywheel to recover. */
    public boolean isWaitingForRpm() { return stage == Stage.FEEDING && !feedRunning; }

    @Override
    public boolean isFinished() { return stage == Stage.DONE; }

    @Override
    public void end(boolean interrupted) {
        stage = Stage.DONE;
        feedRunning = false;
        robot.transfer.closeGate();
        robot.transfer.stopMotor();
        robot.intake.stop();
        if (idleFlywheelOnEnd) robot.shooter.idle();
    }
}
