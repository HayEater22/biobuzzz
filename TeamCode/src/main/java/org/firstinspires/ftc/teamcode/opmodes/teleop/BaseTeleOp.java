package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.CommandRunner;
import org.firstinspires.ftc.teamcode.commands.FlowerCommand;
import org.firstinspires.ftc.teamcode.commands.FlywheelCommand;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.commands.TrackGoalCommand;
import org.firstinspires.ftc.teamcode.config.FieldConstants.Alliance;
import org.firstinspires.ftc.teamcode.config.FieldConstants.HiveEnd;
import org.firstinspires.ftc.teamcode.util.MultiTap;

/**
 * Shared TeleOp. Run {@link RedTeleOp} or {@link BlueTeleOp}.
 *
 * Controls (gamepad1):
 *   Left stick      field-oriented movement
 *   Right stick X   rotation
 *   Right trigger   intake
 *   Left trigger    quick tap: flywheel on / off (stays at speed for the current distance)
 *                   hold: shoot. Waits for speed + aim, then gate opens and feeds.
 *                   The flywheel keeps spinning after a shot; tap to turn it off.
 *   Right bumper    flower mech toggle: press to drop down, press again to fold up
 *   A               reject (intake + transfer in reverse)
 *   X               re-zero field-oriented heading (robot's current facing = forward)
 *   X x3 (quick)    relocalize: front bumper rammed against the metal beam at seam (3,W) red /
 *                   (3,Y) blue, facing forward (intake over the beam)
 *   D-pad up / down aim at the REAR / AUDIENCE cell (only when FieldConstants.AUTO_SWITCH_HIVE_END is off)
 * The turret and hood track our HIVE at all times. With AUTO_SWITCH_HIVE_END on (default) the goal
 * switches by itself when the robot crosses the audience/rear center line (Y = 72).
 */
@Config
public abstract class BaseTeleOp extends OpMode {

    public static double TRIGGER_THRESHOLD = 0.2;
    public static double TRIPLE_TAP_WINDOW_SEC = 0.4;
    /** Left trigger held longer than this = shoot; released sooner = flywheel toggle. */
    public static double SHOOT_HOLD_SEC = 0.3;
    /** Test TeleOp only: RPM change per D-pad up/down press. */
    public static double RPM_STEP = 100;

    private final Alliance alliance;
    private final boolean skipReadyCheck;
    private Robot robot;
    private final CommandRunner commands = new CommandRunner();
    private final MultiTap xTripleTap = new MultiTap(3, TRIPLE_TAP_WINDOW_SEC);
    private TrackGoalCommand tracker;
    private ShootCommand shoot;
    private FlywheelCommand flywheel;
    private boolean triggerWasDown = false;
    private final ElapsedTime triggerTimer = new ElapsedTime();
    private FlowerCommand flower;

    protected BaseTeleOp(Alliance alliance) {
        this(alliance, false);
    }

    /** @param skipReadyCheck true = shooting feeds without waiting for flywheel speed or aim (testing only) */
    protected BaseTeleOp(Alliance alliance, boolean skipReadyCheck) {
        this.alliance = alliance;
        this.skipReadyCheck = skipReadyCheck;
    }

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(hardwareMap);
        robot.setAlliance(alliance);
        tracker = new TrackGoalCommand(robot);
        shoot = skipReadyCheck ? ShootCommand.whileHeldNoWait(robot, tracker)
                               : ShootCommand.whileHeld(robot, tracker);
        flywheel = new FlywheelCommand(robot, tracker);
        flower = FlowerCommand.whileHeld(robot.flower);
        telemetry.addData("Alliance", alliance);
        telemetry.update();
    }

    @Override
    public void start() {
        commands.schedule(tracker);
    }

    @Override
    public void loop() {
        robot.update(telemetry);   // clears bulk cache + runs every subsystem's periodic()

        // ---- Drive ----
        robot.drivetrain.drive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

        if (gamepad1.xWasPressed()) {
            robot.drivetrain.resetImu();
            if (xTripleTap.press()) {
                robot.relocalize();
                gamepad1.rumble(300);
            }
        }

        if (dpadRpmControl()) {
            // ---- Testing: manual flywheel RPM on the D-pad ----
            if (gamepad1.dpadUpWasPressed())   nudgeRpm(+RPM_STEP);
            if (gamepad1.dpadDownWasPressed()) nudgeRpm(-RPM_STEP);
            if (gamepad1.dpadLeftWasPressed()) {
                tracker.clearRpmOverride();                               // back to the shot table
                gamepad1.rumble(150);
            }
        } else {
            // ---- Hive target: which end has the upward cell ----
            if (gamepad1.dpadUpWasPressed()) {
                robot.goal.setHiveEnd(HiveEnd.REAR);
                gamepad1.rumble(150);
            }
            if (gamepad1.dpadDownWasPressed()) {
                robot.goal.setHiveEnd(HiveEnd.AUDIENCE);
                gamepad1.rumble(150);
            }
        }

        // ---- Left trigger: tap = flywheel on/off, hold = shoot (has priority over intake/reject) ----
        boolean shootHeld = gamepad1.left_trigger > TRIGGER_THRESHOLD;
        if (shootHeld && !triggerWasDown) triggerTimer.reset();          // just pressed
        if (shootHeld && triggerTimer.seconds() >= SHOOT_HOLD_SEC && !commands.isRunning(shoot)) {
            commands.schedule(flywheel);                                  // stays on after the shot
            commands.schedule(shoot);
        }
        if (!shootHeld && triggerWasDown) {                               // just released
            if (commands.isRunning(shoot)) {
                commands.cancel(shoot);                                   // end of a hold: stop feeding
            } else if (commands.isRunning(flywheel)) {
                commands.cancel(flywheel);                                // quick tap: off
                gamepad1.rumble(150);
            } else {
                commands.schedule(flywheel);                              // quick tap: on
                gamepad1.rumble(150);
            }
        }
        triggerWasDown = shootHeld;

        // ---- Flower mech (toggle) ----
        if (gamepad1.rightBumperWasPressed()) {
            if (commands.isRunning(flower)) commands.cancel(flower);   // fold back up
            else                            commands.schedule(flower); // drop down
        }

        // ---- Intake / reject ----
        if (!shootHeld) {
            if (gamepad1.a) {
                robot.intake.reverse();
                robot.transfer.reverse();
            } else if (gamepad1.right_trigger > TRIGGER_THRESHOLD) {
                robot.intake.intake();
                robot.transfer.stopMotor();
            } else {
                robot.intake.stop();
                robot.transfer.stopMotor();
            }
        }

        commands.run();

        // ---- Driver info ----
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Target", "%s hive, %s end (%.0f, %.0f)", robot.goal.getAlliance(),
                robot.goal.getHiveEnd(), robot.goal.getGoalX(), robot.goal.getGoalY());
        if (tracker.getSolution() != null) {
            telemetry.addData("Distance", "%.1f in", tracker.getSolution().distanceIn);
        }
        telemetry.addData("Flywheel", commands.isRunning(flywheel)
                ? (robot.shooter.isAtSpeed() ? "ON, at speed" : "ON, spinning up") : "off");
        if (dpadRpmControl() && tracker.getSolution() != null) {
            telemetry.addData("RPM", "%.0f (%s)", tracker.getSolution().rpm,
                    tracker.hasRpmOverride() ? "manual: D-pad up/down, left = table" : "shot table");
        }
        telemetry.addData("Shoot", commands.isRunning(shoot) ? shoot.getStage() : "-");
        telemetry.addData("Aimed", tracker.isAimed());
        telemetry.update();
    }

    /** true = D-pad up/down sets the flywheel RPM instead of the hive end (test TeleOp only). */
    protected boolean dpadRpmControl() { return false; }

    /** First press starts from the RPM currently in use, so the wheel doesn't jump. */
    private void nudgeRpm(double delta) {
        if (tracker.getSolution() == null) return;
        tracker.setRpmOverride(tracker.getSolution().rpm + delta);
    }

    @Override
    public void stop() {
        commands.cancelAll();
        robot.stopAll();
    }
}
