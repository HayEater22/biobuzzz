package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.FieldConstants.Alliance;

/**
 * Same as Blue TeleOp, but no odometry for shooting: turret locked forward (aim by turning the robot),
 * fixed RPM + hood, and it feeds anyway if the flywheel isn't at speed in time.
 */
@TeleOp(name = "Blue TeleOp (Turret Locked)", group = "Blue")
public class BlueTurretLockedTeleOp extends BaseTeleOp {
    public BlueTurretLockedTeleOp() { super(Alliance.BLUE); }

    @Override
    protected boolean lockTurret() { return true; }
}
