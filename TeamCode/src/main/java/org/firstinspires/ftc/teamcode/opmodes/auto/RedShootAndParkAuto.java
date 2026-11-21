package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.config.FieldConstants.Alliance;

@Autonomous(name = "Red Shoot and Park", group = "Red", preselectTeleOp = "Red TeleOp")
@Config
public class RedShootAndParkAuto extends BaseShootAndParkAuto {

    // ---- Tuning (inches / degrees; 0..144 frame, (0,0) = red garden corner) ---- TUNE: measure
    public static double START_X = 0, START_Y = 0, START_HEADING = 0;
    public static double PARK_X = 24, PARK_Y = 0, PARK_HEADING = 0;

    public RedShootAndParkAuto() {
        super(Alliance.RED, START_X, START_Y, START_HEADING, PARK_X, PARK_Y, PARK_HEADING);
    }
}
