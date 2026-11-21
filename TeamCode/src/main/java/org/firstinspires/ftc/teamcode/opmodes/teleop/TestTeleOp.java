package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.MatchState;

/**
 * Testing TeleOp: same controls as the normal TeleOp, but holding left trigger feeds
 * immediately WITHOUT waiting for flywheel RPM or turret/hood aim.
 * Uses the last selected alliance.
 *
 * Flywheel RPM on the D-pad (replaces the hive-end buttons here):
 *   D-pad up / down   RPM +/- BaseTeleOp.RPM_STEP (first press starts from the current RPM)
 *   D-pad left        back to the shot table's RPM
 * The flywheel still turns on/off with a left-trigger tap. Turret and hood keep auto-aiming.
 */
@TeleOp(name = "11-test tele", group = "Test")
public class TestTeleOp extends BaseTeleOp {
    public TestTeleOp() {
        super(MatchState.alliance, true);
    }

    @Override
    protected boolean dpadRpmControl() { return true; }
}
