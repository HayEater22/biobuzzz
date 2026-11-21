package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;

/**
 * Flywheel tuner. Robot does not drive. All speeds are in RPM.
 *
 * Gains live in FTC Dashboard -> Config -> "Shooter" (kS, kV, kP, kI, ...) and apply instantly.
 * This OpMode's settings are in Config -> "FlywheelTuner".
 * Graph tab: plot target, velocity, error, power, voltage.
 *
 * MODE:
 *   CONSTANT  hold TARGET_VELOCITY
 *   STEP      switch between TARGET_VELOCITY and LOW_VELOCITY every STEP_SECONDS
 *
 * Controls (gamepad1):
 *   A             spin up
 *   B             stop (wheel coasts)
 *   Right bumper  hold to feed (gate open, transfer + intake run)
 */
@Config
@TeleOp(name = "Flywheel Tuner", group = "Test")
public class FlywheelTuner extends OpMode {

    public enum Mode { CONSTANT, STEP }

    public static double TARGET_VELOCITY = 3000;
    public static double LOW_VELOCITY = 2000;
    public static double STEP_SECONDS = 2.0;
    public static Mode MODE = Mode.CONSTANT;

    private Robot robot;
    private Telemetry driverStation;
    private final ElapsedTime stepTimer = new ElapsedTime();
    private boolean spinning = false;

    @Override
    public void init() {
        driverStation = telemetry;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(hardwareMap);
        telemetry.addLine("Press START, then A to spin up.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // ---- Inputs ----
        if (gamepad1.aWasPressed()) {
            spinning = true;
            stepTimer.reset();
        }
        if (gamepad1.bWasPressed()) spinning = false;

        // ---- Commands ----
        double target = 0;
        if (spinning) {
            boolean low = MODE == Mode.STEP && STEP_SECONDS > 0
                    && ((int) (stepTimer.seconds() / STEP_SECONDS)) % 2 == 1;
            target = low ? LOW_VELOCITY : TARGET_VELOCITY;
        }
        robot.shooter.setTargetRpm(target); // 0 = no power, wheel coasts

        if (gamepad1.right_bumper) {
            robot.transfer.openGate();
            robot.transfer.feed();
            robot.intake.intake();
        } else {
            robot.transfer.closeGate();
            robot.transfer.stopMotor();
            robot.intake.stop();
        }

        robot.update(telemetry);

        // ---- Telemetry (graph) ----
        double velocity = robot.shooter.getRpm();
        double setpoint = robot.shooter.getSetpointRpm(); // ramped target the controller is chasing
        telemetry.addData("target", setpoint);
        telemetry.addData("velocity", velocity);
        telemetry.addData("error", setpoint - velocity);
        telemetry.addData("power", robot.shooter.getPowerOutput());
        telemetry.addData("voltage", robot.shooter.getBatteryVoltage());

        // Driver Station only (keeps the Dashboard graph to the numbers above)
        driverStation.addData("At speed", robot.shooter.isAtSpeed() ? "YES" : "no");
        driverStation.addData("Mode", "%s, %s", MODE, spinning ? "spinning (B = stop)" : "stopped (A = spin)");

        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stopAll();
    }
}
