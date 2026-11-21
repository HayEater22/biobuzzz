package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.FieldConstants;
import org.firstinspires.ftc.teamcode.config.FieldConstants.Alliance;
import org.firstinspires.ftc.teamcode.config.FieldConstants.HiveEnd;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Localization;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.targeting.GoalSelector;
import org.firstinspires.ftc.teamcode.targeting.ShotSolution;
import org.firstinspires.ftc.teamcode.util.BulkCache;

/**
 * Odometry + aiming check with ONLY the drive motors and Pinpoint configured.
 * The turret, shooter and hood are not touched; it prints what they WOULD be told.
 *
 * Controls (gamepad1):
 *   Left stick / right stick X   field-oriented drive
 *   X                            re-zero driving heading
 *   B                            set pose to (0, 0, 0°) for pod checks
 *   D-pad left / right           target Blue / Red hive
 *   D-pad up / down              hive upward cell: REAR / AUDIENCE end
 *
 * Pod checks (push the robot by hand, starting from B = zero):
 *   forward  -> X goes up      left -> Y goes up      turn CCW -> heading goes up
 *   push exactly 48 in forward -> X reads ~48
 *
 * Pose is in the 0..144 field frame (see FieldConstants). Pressing B puts the robot at (0,0),
 * i.e. the red garden corner, for pod checks only.
 *
 * Dashboard (192.168.43.1:8080/dash) Field view: robot, heading line, hive cell, and the aim line.
 */
@TeleOp(name = "Odometry Aim Test", group = "Test")
public class OdometryAimTest extends OpMode {

    private BulkCache bulkCache;
    private Drivetrain drivetrain;
    private Localization localization;
    private GoalSelector goal;

    @Override
    public void init() {
        bulkCache = new BulkCache(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap);
        localization = new Localization(drivetrain);
        goal = new GoalSelector(Alliance.BLUE);
        telemetry.addLine("Only drive motors + Pinpoint are used. Turret/shooter can be unplugged.");
        telemetry.update();
    }

    @Override
    public void loop() {
        bulkCache.clear();
        drivetrain.periodic(telemetry);   // follower.update(): reads Pinpoint + drives

        // ---- Controls ----
        drivetrain.drive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        if (gamepad1.xWasPressed()) drivetrain.resetImu();
        if (gamepad1.bWasPressed()) {
            localization.setPose(0, 0, 0);
            drivetrain.resetImu();
        }
        if (gamepad1.dpad_left)  goal.setAlliance(Alliance.BLUE);
        if (gamepad1.dpad_right) goal.setAlliance(Alliance.RED);
        if (gamepad1.dpadUpWasPressed())   goal.setHiveEnd(HiveEnd.REAR);
        if (gamepad1.dpadDownWasPressed()) goal.setHiveEnd(HiveEnd.AUDIENCE);

        // ---- What the robot would aim at ----
        double x = localization.getX(), y = localization.getY(), h = localization.getHeading();
        goal.updateFromPose(y);
        ShotSolution shot = ShotSolution.compute(x, y, h, goal);
        boolean inRange = shot.turretAngleDeg >= Turret.MIN_ANGLE_DEG && shot.turretAngleDeg <= Turret.MAX_ANGLE_DEG;

        telemetry.addLine("--- Pose ---");
        telemetry.addData("X (in)", "%.2f", x);
        telemetry.addData("Y (in)", "%.2f", y);
        telemetry.addData("Heading (deg)", "%.2f", Math.toDegrees(h));
        telemetry.addLine("--- Target ---");
        telemetry.addData("Target", "%s hive, %s end at (%.1f, %.1f)", goal.getAlliance(), goal.getHiveEnd(),
                goal.getGoalX(), goal.getGoalY());
        telemetry.addData("Distance (in)", "%.1f", shot.distanceIn);
        telemetry.addData("Turret angle (deg)", "%.1f  %s", shot.turretAngleDeg, inRange ? "in range" : "OUT OF RANGE");
        telemetry.addData("Shooter RPM", "%.0f", shot.rpm);
        telemetry.addData("Hood", "%.2f", shot.hood);
        telemetry.addLine("B = zero pose · X = re-zero driving · D-pad L/R = Blue/Red · U/D = hive end");
        telemetry.update();

        // ---- Dashboard field view ----
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("x", x);
        packet.put("y", y);
        packet.put("headingDeg", Math.toDegrees(h));
        packet.put("distance", shot.distanceIn);
        packet.put("turretAngleDeg", shot.turretAngleDeg);
        Canvas field = packet.fieldOverlay();
        // Dashboard draws with (0,0) at field center; our frame has (0,0) in the red garden corner.
        double c = FieldConstants.FIELD_SIZE_IN / 2.0;
        double dx = x - c, dy = y - c;
        double aim = h + Math.toRadians(shot.turretAngleDeg);   // field direction the turret would point
        field.setStroke("#0d5fa8").strokeCircle(dx, dy, 9)
             .strokeLine(dx, dy, dx + 12 * Math.cos(h), dy + 12 * Math.sin(h));
        field.setStroke(inRange ? "#1d7a46" : "#c0392b")
             .strokeLine(dx, dy, dx + shot.distanceIn * Math.cos(aim), dy + shot.distanceIn * Math.sin(aim));
        field.setFill(goal.getAlliance() == Alliance.BLUE ? "#1e5bd8" : "#d02f2f")
             .fillCircle(goal.getGoalX() - c, goal.getGoalY() - c, 4);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    @Override
    public void stop() {
        drivetrain.stop();
    }
}
