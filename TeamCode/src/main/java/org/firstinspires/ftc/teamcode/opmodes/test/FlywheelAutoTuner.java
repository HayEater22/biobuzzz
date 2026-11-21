package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.testing.FlywheelAutoTune;

import java.util.List;

/**
 * Flywheel auto tuner: tunes Shooter kS, kV, kP, kI and the boost by itself. Robot does not drive.
 *
 * Settings: FTC Dashboard -> Config -> "FlywheelAutoTune" (TUNE_RPM = the RPM you shoot at,
 * which phases to run, boost thresholds to try).
 *
 * Controls (gamepad1):
 *   A   start / continue after a prompt ("Load 4 balls, then press A")
 *   B   stop now (wheel coasts, feeding stops)
 *   X   skip the current phase (keeps the old values)
 *
 * Takes ~2 min plus one 4-ball reload per boost setting. Results apply live, but are lost on
 * restart: copy the final values into Shooter.java.
 */
@TeleOp(name = "Flywheel Auto Tuner", group = "Test")
public class FlywheelAutoTuner extends OpMode {

    private Robot robot;
    private FlywheelAutoTune tuner;
    private Telemetry driverStation;

    @Override
    public void init() {
        driverStation = telemetry;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(hardwareMap);
        tuner = new FlywheelAutoTune(robot);
        telemetry.addLine("Robot stays still; only the flywheel and feeder run.");
        telemetry.addLine("Gate closed, no balls loaded. Press START, then A.");
        telemetry.update();
    }

    @Override
    public void loop() {
        robot.update(telemetry);   // clears bulk cache + runs every subsystem's periodic()

        if (gamepad1.aWasPressed()) tuner.confirm();
        if (gamepad1.bWasPressed()) tuner.abort("stopped with B");
        if (gamepad1.xWasPressed()) tuner.skip();

        tuner.update();

        // ---- Dashboard graph ----
        telemetry.addData("target", robot.shooter.getSetpointRpm());
        telemetry.addData("rpm", robot.shooter.getRpm());
        telemetry.addData("power", robot.shooter.getPowerOutput());
        telemetry.addData("voltage", robot.shooter.getBatteryVoltage());
        telemetry.addData("phase", tuner.getPhase().ordinal());

        // ---- Driver Station ----
        driverStation.addData("Phase", tuner.getPhase());
        driverStation.addData("Status", tuner.getStatus());
        if (tuner.getPrompt() != null) {
            driverStation.addLine();
            driverStation.addLine(">>> " + tuner.getPrompt() + " <<<");
        }
        if (!tuner.getResults().isEmpty()) {
            driverStation.addLine();
            driverStation.addLine("---- Copy into Shooter.java ----");
            for (String line : tuner.getResults()) driverStation.addLine(line);
        }
        List<String> log = tuner.getLog();
        if (!log.isEmpty()) {
            driverStation.addLine();
            for (int i = Math.max(0, log.size() - 8); i < log.size(); i++) driverStation.addLine(log.get(i));
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stopAll();
    }
}
