package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.testing.MotorProbe;

/**
 * Spin one motor at a time to check wiring vs. the robot config.
 *
 * Controls (gamepad1):
 *   D-pad left / right   pick a motor (stops the previous one)
 *   Left stick Y         run the picked motor (push up = forward)
 *
 * Shows the hub + port the config says each motor is on, and the current it draws.
 * Power sent but ~0 A current = nothing is plugged into that port.
 * A different motor spins = that motor's cable is in the wrong port.
 * Lift the robot off the ground before testing drive motors.
 */
@TeleOp(name = "Motor Tester", group = "Test")
public class MotorTester extends OpMode {

    private MotorProbe probe;

    @Override
    public void init() {
        probe = new MotorProbe(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.dpadRightWasPressed()) probe.selectNext(1);
        if (gamepad1.dpadLeftWasPressed())  probe.selectNext(-1);
        probe.run(-gamepad1.left_stick_y);

        MotorProbe.Entry picked = probe.getSelected();
        telemetry.addData("PICKED", picked.name);
        telemetry.addData("Port", picked.getPort());
        telemetry.addData("Power", "%.2f", picked.getPower());
        telemetry.addData("Current", "%.2f A", picked.getCurrentAmps());
        telemetry.addLine();
        for (int i = 0; i < probe.getEntries().size(); i++) {
            MotorProbe.Entry e = probe.getEntries().get(i);
            telemetry.addData((i == probe.getSelectedIndex() ? "> " : "  ") + e.name, e.getPort());
        }
        telemetry.addLine();
        telemetry.addLine("D-pad L/R pick · left stick Y run");
        telemetry.update();
    }

    @Override
    public void stop() {
        probe.stopAll();
    }
}
