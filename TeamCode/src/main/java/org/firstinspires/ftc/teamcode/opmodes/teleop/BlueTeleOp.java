package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.FieldConstants.Alliance;

@TeleOp(name = "Blue TeleOp", group = "Blue")
public class BlueTeleOp extends BaseTeleOp {
    public BlueTeleOp() { super(Alliance.BLUE); }
}
