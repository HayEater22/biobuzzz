package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Turret;

/**
 * Calibrates the turret's angle -> servo conversion (Turret.GEAR_RATIO, Turret.INVERTED, Turret.CENTER_POSITION).
 * Only the turret servos are used; the robot does not drive.
 *
 * Controls (gamepad1):
 *   D-pad up      0°   (straight forward)
 *   D-pad left    +90° (should point straight LEFT)
 *   D-pad right   -90° (should point straight RIGHT)
 *   Bumpers       ±5°
 *
 * Steps:
 *   1. D-pad up. Not straight forward? Adjust Turret.CENTER_POSITION on the Dashboard until it is.
 *   2. D-pad left. If it went RIGHT, set Turret.INVERTED = true.
 *   3. Measure how far it actually turned (put a square / protractor on it).
 *      New GEAR_RATIO = GEAR_RATIO * 90 / (actual degrees turned). Repeat until +90 and -90 are both square.
 *   4. Copy the final values into Turret.java.
 */
@TeleOp(name = "Turret Calibration", group = "Test")
public class TurretCalibration extends OpMode {

    private Turret turret;
    private double angle = 0.0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        turret = new Turret(hardwareMap);
        telemetry.addLine("Press START. Turret will go to 0° (forward).");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.dpadUpWasPressed())    angle = 0.0;
        if (gamepad1.dpadLeftWasPressed())  angle = 90.0;
        if (gamepad1.dpadRightWasPressed()) angle = -90.0;
        if (gamepad1.rightBumperWasPressed()) angle += 5.0;
        if (gamepad1.leftBumperWasPressed())  angle -= 5.0;

        turret.setAngle(angle);
        turret.periodic(telemetry);

        telemetry.addData("Commanded", "%.0f°  (%s)", angle,
                angle > 0 ? "should point LEFT" : angle < 0 ? "should point RIGHT" : "should point FORWARD");
        telemetry.addData("GEAR_RATIO", "%.3f", Turret.GEAR_RATIO);
        telemetry.addData("INVERTED", Turret.INVERTED);
        telemetry.addData("CENTER_POSITION", "%.3f", Turret.CENTER_POSITION);
        telemetry.addData("Max reach", "±%.0f°", Turret.reachDeg());
        telemetry.addLine("Fix: GEAR_RATIO = GEAR_RATIO * 90 / actualDegrees");
        telemetry.update();
    }
}
