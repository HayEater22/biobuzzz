package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.FieldConstants.Alliance;

@TeleOp(name = "Red TeleOp", group = "Red")
public class RedTeleOp extends BaseTeleOp {
    public RedTeleOp() { super(Alliance.RED); }
}
