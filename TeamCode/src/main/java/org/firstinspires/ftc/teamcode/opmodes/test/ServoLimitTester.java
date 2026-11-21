package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.testing.ServoProbe;

import java.util.HashMap;
import java.util.Map;

/**
 * Find each servo's safe min / max position.
 *
 * Controls (gamepad1):
 *   Right / left bumper   pick next / previous servo
 *   D-pad up / down       +0.01 / -0.01 (small step)
 *   D-pad right / left    +0.05 / -0.05 (big step)
 *   B                     make the picked servo go limp
 *   Y                     clear the recorded min/max for the picked servo
 *
 * Telemetry shows the live position plus the lowest and highest position reached for every servo,
 * so you can copy them straight into the code as limits.
 *
 * The FIRST move sends the servo to about 0.5, which can be a big jump: keep hands clear.
 * If a servo buzzes or strains, it is pushing on a hard stop: back off one step. That's your limit.
 * "turret (1+2)" moves both turret servos together. "turret1 ALONE" / "turret2 ALONE" move one
 * servo, to find which port it's on or check its direction: ONLY with the turret gear off.
 */
@TeleOp(name = "Servo Limit Tester", group = "Test")
public class ServoLimitTester extends OpMode {

    public static double SMALL_STEP = 0.01;
    public static double BIG_STEP = 0.05;

    private ServoProbe probe;
    private final Map<String, double[]> limits = new HashMap<>();   // name -> {min, max}

    @Override
    public void init() {
        probe = new ServoProbe(hardwareMap);
    }

    @Override
    public void loop() {
        if (probe.isEmpty()) {
            telemetry.addLine("No robot servos found in the config.");
            telemetry.addLine("Add turret1, turret2, hood, gate, flowerLeft or flowerRight.");
            telemetry.update();
            return;
        }

        if (gamepad1.rightBumperWasPressed()) probe.selectNext(1);
        if (gamepad1.leftBumperWasPressed())  probe.selectNext(-1);

        double step = 0;
        if (gamepad1.dpadUpWasPressed())    step += SMALL_STEP;
        if (gamepad1.dpadDownWasPressed())  step -= SMALL_STEP;
        if (gamepad1.dpadRightWasPressed()) step += BIG_STEP;
        if (gamepad1.dpadLeftWasPressed())  step -= BIG_STEP;
        if (step != 0) {
            probe.nudge(step);
            record(probe.getSelected());
        }

        if (gamepad1.bWasPressed()) probe.release();
        if (gamepad1.yWasPressed()) limits.remove(probe.getSelected().name);

        // ---- Display ----
        ServoProbe.Entry picked = probe.getSelected();
        telemetry.addData("PICKED", picked.name);
        telemetry.addData("Port", picked.getPort());
        if (picked.name.endsWith("ALONE")) telemetry.addLine("!! Only with the turret gear OFF, or the servos will fight !!");
        telemetry.addData("Position", "%.3f%s", picked.getPosition(),
                picked.hasMoved() ? "" : "  (not moving yet: press D-pad)");
        double[] pl = limits.get(picked.name);
        if (pl != null) telemetry.addData("Min / Max", "%.3f / %.3f", pl[0], pl[1]);
        telemetry.addLine();

        telemetry.addLine("ALL SERVOS  (pos | min - max)");
        for (int i = 0; i < probe.getEntries().size(); i++) {
            ServoProbe.Entry e = probe.getEntries().get(i);
            double[] l = limits.get(e.name);
            String range = l == null ? "not tested" : String.format("%.3f - %.3f", l[0], l[1]);
            telemetry.addData((i == probe.getSelectedIndex() ? "> " : "  ") + e.name, "%s | %s",
                    e.hasMoved() ? String.format("%.3f", e.getPosition()) : "off", range);
        }
        telemetry.addLine();
        telemetry.addLine("Bumpers pick · D-pad ↑↓ ±0.01 · D-pad →← ±0.05 · B limp · Y clear");
        telemetry.update();
    }

    private void record(ServoProbe.Entry e) {
        double pos = e.getPosition();
        double[] l = limits.get(e.name);
        if (l == null) {
            limits.put(e.name, new double[]{pos, pos});
        } else {
            l[0] = Math.min(l[0], pos);
            l[1] = Math.max(l[1], pos);
        }
    }
}
