package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.config.FieldConstants.Alliance;

@Autonomous(name = "Blue Shoot and Park", group = "Blue", preselectTeleOp = "Blue TeleOp")
@Config
public class BlueShootAndParkAuto extends BaseShootAndParkAuto {

    // ---- Tuning (inches / degrees; 0..144 frame, (0,0) = red garden corner) ---- TUNE: measure
    public static double START_X = 0, START_Y = 0, START_HEADING = 0;
    public static double PARK_X = 24, PARK_Y = 0, PARK_HEADING = 0;

    public BlueShootAndParkAuto() {
        super(Alliance.BLUE, START_X, START_Y, START_HEADING, PARK_X, PARK_Y, PARK_HEADING);
    }
}
