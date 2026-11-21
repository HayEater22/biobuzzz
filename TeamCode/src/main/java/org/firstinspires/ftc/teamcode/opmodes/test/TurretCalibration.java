package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.FieldConstants;
import org.firstinspires.ftc.teamcode.config.FieldConstants.Alliance;
import org.firstinspires.ftc.teamcode.config.MatchState;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Localization;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.targeting.GoalSelector;
import org.firstinspires.ftc.teamcode.targeting.ShotSolution;
import org.firstinspires.ftc.teamcode.util.BulkCache;

import java.util.ArrayList;
import java.util.List;

/**
 * Two modes, switched with A:
 *
 * ===== CALIBRATE (starts here) =====
 * Calibrates the turret's angle -> servo conversion (Turret.GEAR_RATIO, Turret.INVERTED, Turret.CENTER_POSITION).
 * The robot does not drive in this mode.
 *   D-pad up      0°   (straight forward)
 *   D-pad left    +90° (should point straight LEFT)
 *   D-pad right   -90° (should point straight RIGHT)
 *   Bumpers       ±5°
 * Steps:
 *   1. D-pad up. Not straight forward? Adjust Turret.CENTER_POSITION on the Dashboard until it is.
 *   2. D-pad left. If it went RIGHT, flip Turret.INVERTED.
 *   3. Measure how far it actually turned (put a square / protractor on it).
 *      New GEAR_RATIO = GEAR_RATIO * 90 / (actual degrees turned). Repeat until +90 and -90 are both square.
 *   4. Copy the final values into Turret.java (Dashboard changes are lost when the robot restarts!).
 *
 * ===== AIM CHECK =====
 * The turret aims at the goal from odometry, exactly like TeleOp. Park the robot, STOP, and check
 * whether the turret points at the goal. If not, nudge it with the bumpers until it does: the
 * nudge ("trim") is the aim error at that spot. Press B to log the spot. Log 5+ spots, then read the pattern:
 *   - Same trim everywhere                 -> Turret.CENTER_POSITION is off
 *   - Trim grows with turret angle         -> Turret.GEAR_RATIO is off (or servo not linear)
 *   - Trim depends on where you stand      -> ShotSolution.TURRET_OFFSET_X / _Y or the goal coordinates are off
 * Controls:
 *   Sticks           field-oriented drive
 *   X                relocalize (front bumper rammed on the beam, same spot as TeleOp's triple-tap X)
 *   D-pad left/right Blue / Red alliance
 *   Bumpers          trim ±1° (right = turn more RIGHT, left = turn more LEFT)
 *   Y                reset trim to 0
 *   B                log this spot (pose, aim angle, trim)
 */
@TeleOp(name = "Turret Calibration", group = "Test")
public class TurretCalibration extends OpMode {

    private static final int MAX_LOG_ROWS = 8;

    private BulkCache bulkCache;
    private Drivetrain drivetrain;
    private Localization localization;
    private Turret turret;
    private GoalSelector goal;

    private boolean aimMode = false;
    private double calAngle = 0.0;
    private double trimDeg = 0.0;
    private final List<String> log = new ArrayList<>();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        bulkCache = new BulkCache(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap);
        localization = new Localization(drivetrain);
        turret = new Turret(hardwareMap);
        goal = new GoalSelector(MatchState.alliance);
        setAlliance(MatchState.alliance);
        telemetry.addLine("Press START. Turret will go to 0° (forward). A = switch CALIBRATE / AIM CHECK.");
        telemetry.update();
    }

    @Override
    public void loop() {
        bulkCache.clear();
        drivetrain.periodic(telemetry);   // reads the Pinpoint

        if (gamepad1.aWasPressed()) aimMode = !aimMode;

        if (aimMode) aimCheckLoop();
        else         calibrateLoop();

        turret.periodic(telemetry);
        telemetry.update();
    }

    // ---------------------------------------------------------------- CALIBRATE

    private void calibrateLoop() {
        drivetrain.drive(0, 0, 0);
        if (gamepad1.dpadUpWasPressed())    calAngle = 0.0;
        if (gamepad1.dpadLeftWasPressed())  calAngle = 90.0;
        if (gamepad1.dpadRightWasPressed()) calAngle = -90.0;
        if (gamepad1.rightBumperWasPressed()) calAngle += 5.0;
        if (gamepad1.leftBumperWasPressed())  calAngle -= 5.0;

        turret.setAngle(calAngle);

        telemetry.addLine("=== CALIBRATE ===  (A = aim check)");
        telemetry.addData("Commanded", "%.0f°  (%s)", calAngle,
                calAngle > 0 ? "should point LEFT" : calAngle < 0 ? "should point RIGHT" : "should point FORWARD");
        telemetry.addData("GEAR_RATIO", "%.3f", Turret.GEAR_RATIO);
        telemetry.addData("INVERTED", Turret.INVERTED);
        telemetry.addData("CENTER_POSITION", "%.3f", Turret.CENTER_POSITION);
        telemetry.addData("Max reach", "±%.0f°", Turret.reachDeg());
        telemetry.addLine("Fix: GEAR_RATIO = GEAR_RATIO * 90 / actualDegrees");
        telemetry.addLine("Copy final values into Turret.java!");
    }

    // ---------------------------------------------------------------- AIM CHECK

    private void aimCheckLoop() {
        drivetrain.drive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

        if (gamepad1.xWasPressed()) {
            Alliance a = goal.getAlliance();
            localization.setPose(FieldConstants.relocalizeX(a), FieldConstants.relocalizeY(a),
                    FieldConstants.relocalizeHeadingDeg(a));
            gamepad1.rumble(300);
        }
        if (gamepad1.dpadLeftWasPressed())  setAlliance(Alliance.BLUE);
        if (gamepad1.dpadRightWasPressed()) setAlliance(Alliance.RED);
        if (gamepad1.leftBumperWasPressed())  trimDeg += 1.0;   // + = counter-clockwise = more LEFT
        if (gamepad1.rightBumperWasPressed()) trimDeg -= 1.0;
        if (gamepad1.yWasPressed()) trimDeg = 0.0;

        double x = localization.getX(), y = localization.getY(), h = localization.getHeading();
        goal.updateFromPose(y);
        ShotSolution shot = ShotSolution.compute(x, y, h, goal);
        turret.setAngle(shot.turretAngleDeg + trimDeg);

        if (gamepad1.bWasPressed()) {
            log.add(0, String.format("(%.0f,%.0f) hd %.0f° | aim %.0f° | dist %.0f | trim %+.0f°",
                    x, y, Math.toDegrees(h), shot.turretAngleDeg, shot.distanceIn, trimDeg));
            if (log.size() > MAX_LOG_ROWS) log.remove(log.size() - 1);
            gamepad1.rumble(150);
        }

        telemetry.addLine("=== AIM CHECK ===  (A = calibrate)");
        telemetry.addData("Pose", "X %.1f  Y %.1f  heading %.1f°", x, y, Math.toDegrees(h));
        telemetry.addData("Target", "%s %s end at (%.1f, %.1f)", goal.getAlliance(), goal.getHiveEnd(),
                goal.getGoalX(), goal.getGoalY());
        telemetry.addData("Distance", "%.1f in", shot.distanceIn);
        telemetry.addData("Aim angle (calc)", "%.1f°  %s", shot.turretAngleDeg, turret.isAtLimit() ? "OUT OF REACH" : "");
        telemetry.addData("Trim (error)", "%+.0f°  %s", trimDeg,
                trimDeg > 0 ? "(had to turn more LEFT)" : trimDeg < 0 ? "(had to turn more RIGHT)" : "");
        telemetry.addData("Turret offset", "X %.2f  Y %.2f in", ShotSolution.TURRET_OFFSET_X, ShotSolution.TURRET_OFFSET_Y);
        telemetry.addLine("STOP the robot before judging. Bumpers = trim, Y = clear trim, B = log, X = relocalize");
        telemetry.addLine("--- Log (newest first) ---");
        for (String row : log) telemetry.addLine(row);
    }

    private void setAlliance(Alliance a) {
        MatchState.alliance = a;
        goal.setAlliance(a);
        drivetrain.setDriverForwardHeading(FieldConstants.driverHeadingDeg(a));
    }

    @Override
    public void stop() {
        drivetrain.stop();
    }
}
