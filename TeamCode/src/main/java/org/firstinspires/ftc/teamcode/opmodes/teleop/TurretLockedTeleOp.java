package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.FieldConstants.Alliance;

/**
 * Turret Locked TeleOp (runs as the BLUE alliance: field-oriented forward, relocalize spot and
 * D-pad field headings are Blue's). No X/Y odometry for the shot: fixed RPM + hood from
 * LockedShotConfig, D-pad turret field headings, one ball at a time.
 */
@TeleOp(name = "TeleOp (Turret Locked)")
public class TurretLockedTeleOp extends BaseTeleOp {
    public TurretLockedTeleOp() { super(Alliance.BLUE); }

    @Override
    protected boolean lockTurret() { return true; }
}
