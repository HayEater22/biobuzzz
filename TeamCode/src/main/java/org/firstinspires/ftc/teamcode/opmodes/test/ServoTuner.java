package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.testing.ServoProbe;

/**
 * Find servo positions one servo at a time.
 *
 * Controls (gamepad1):
 *   D-pad left / right     pick a servo
 *   Right / left bumper    +0.01 / -0.01 (small step)
 *   D-pad up / down        +0.05 / -0.05 (big step)
 *   B                      make the picked servo go limp
 *
 * Nothing moves until you press a move button. The FIRST press sends the servo to about 0.5,
 * which can be a big jump: take horns/linkages off or keep hands clear.
 * If a servo buzzes or strains, it is pushing on a hard stop: back off right away.
 */
@TeleOp(name = "Servo Tuner", group = "Test")
public class ServoTuner extends OpMode {

    private ServoProbe probe;

    @Override
    public void init() {
        probe = new ServoProbe(hardwareMap);
    }

    @Override
    public void loop() {
        if (probe.isEmpty()) {
            telemetry.addLine("No robot servos found in the config.");
            telemetry.addLine("Add turret1, turret2, hood, gate, flowerLeft or flowerRight to the robot config.");
            telemetry.update();
            return;
        }

        if (gamepad1.dpadRightWasPressed()) probe.selectNext(1);
        if (gamepad1.dpadLeftWasPressed())  probe.selectNext(-1);
        if (gamepad1.rightBumperWasPressed()) probe.nudge(0.01);
        if (gamepad1.leftBumperWasPressed())  probe.nudge(-0.01);
        if (gamepad1.dpadUpWasPressed())   probe.nudge(0.05);
        if (gamepad1.dpadDownWasPressed()) probe.nudge(-0.05);
        if (gamepad1.bWasPressed()) probe.release();

        ServoProbe.Entry picked = probe.getSelected();
        telemetry.addData("PICKED", "%s = %.3f%s", picked.name, picked.getPosition(),
                picked.hasMoved() ? "" : "  (not moving yet)");
        telemetry.addData("Port", picked.getPort());
        if (picked.name.endsWith("ALONE")) telemetry.addLine("!! Only with the turret gear OFF, or the servos will fight !!");
        telemetry.addLine();
        for (int i = 0; i < probe.getEntries().size(); i++) {
            ServoProbe.Entry e = probe.getEntries().get(i);
            telemetry.addData((i == probe.getSelectedIndex() ? "> " : "  ") + e.name, "%.3f%s",
                    e.getPosition(), e.hasMoved() ? "" : "  (off)");
        }
        telemetry.addLine();
        telemetry.addLine("D-pad L/R pick · bumpers ±0.01 · D-pad up/down ±0.05 · B limp");
        telemetry.update();
    }
}
