package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.CommandRunner;
import org.firstinspires.ftc.teamcode.commands.DriveToPoseCommand;
import org.firstinspires.ftc.teamcode.commands.SequentialCommand;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.commands.TrackGoalCommand;
import org.firstinspires.ftc.teamcode.config.FieldConstants;
import org.firstinspires.ftc.teamcode.config.FieldConstants.Alliance;

/** Shoots preloads from the start position, then parks. Run {@link RedShootAndParkAuto} or {@link BlueShootAndParkAuto}. */
@Config
public abstract class BaseShootAndParkAuto extends LinearOpMode {

    public static double READY_TIMEOUT_SEC = 2.0;
    public static double FEED_SEC = 2.0;

    private final Alliance alliance;
    private final double startX, startY, startHeading;
    private final double parkX, parkY, parkHeading;

    /** Poses in inches / degrees, 0..144 field frame with (0,0) in the red garden corner (see FieldConstants). */
    protected BaseShootAndParkAuto(Alliance alliance,
                                   double startX, double startY, double startHeading,
                                   double parkX, double parkY, double parkHeading) {
        this.alliance = alliance;
        this.startX = startX; this.startY = startY; this.startHeading = startHeading;
        this.parkX = parkX; this.parkY = parkY; this.parkHeading = parkHeading;
    }

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot robot = new Robot(hardwareMap);
        robot.setAlliance(alliance);
        robot.goal.setHiveEnd(FieldConstants.startingHiveEnd(alliance));   // fresh match: starting cell

        telemetry.addData("Alliance", alliance);
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        robot.localization.setPose(startX, startY, startHeading);

        CommandRunner commands = new CommandRunner();
        TrackGoalCommand tracker = new TrackGoalCommand(robot);
        SequentialCommand routine = new SequentialCommand(
                new ShootCommand(robot, tracker, READY_TIMEOUT_SEC, FEED_SEC),
                new DriveToPoseCommand(robot, parkX, parkY, parkHeading));

        commands.schedule(tracker);
        commands.schedule(routine);

        while (opModeIsActive() && commands.isRunning(routine)) {
            robot.update(telemetry);   // clears bulk cache + runs every subsystem's periodic()
            commands.run();
            telemetry.update();
        }

        commands.cancelAll();
        robot.stopAll();
    }
}
