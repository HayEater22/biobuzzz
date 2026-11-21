package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.FieldConstants.Alliance;

/**
 * Same as Red TeleOp, but no odometry for shooting: turret locked forward (aim by turning the robot),
 * fixed RPM + hood, and it feeds anyway if the flywheel isn't at speed in time.
 */
@TeleOp(name = "Red TeleOp (Turret Locked)", group = "Red")
public class RedTurretLockedTeleOp extends BaseTeleOp {
    public RedTurretLockedTeleOp() { super(Alliance.RED); }

    @Override
    protected boolean lockTurret() { return true; }
}
