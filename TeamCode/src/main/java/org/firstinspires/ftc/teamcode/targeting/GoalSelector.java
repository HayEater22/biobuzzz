package org.firstinspires.ftc.teamcode.targeting;

import org.firstinspires.ftc.teamcode.config.FieldConstants;
import org.firstinspires.ftc.teamcode.config.FieldConstants.Alliance;
import org.firstinspires.ftc.teamcode.config.FieldConstants.HiveEnd;
import org.firstinspires.ftc.teamcode.config.MatchState;

/**
 * Decides where the robot shoots: the upward cell of our alliance's HIVE.
 * The upward cell swaps ends each time the hive tips, so the driver tells us which end is up.
 * The choice is stored in {@link MatchState} so it carries from Autonomous into TeleOp.
 */
public class GoalSelector {

    private Alliance alliance;
    private HiveEnd upEnd;

    public GoalSelector(Alliance alliance) {
        this.alliance = alliance;
        this.upEnd = MatchState.hiveEnd;
    }

    /** Changing alliance resets the hive end to that alliance's starting end. */
    public void setAlliance(Alliance alliance) {
        if (alliance != this.alliance) setHiveEnd(FieldConstants.startingHiveEnd(alliance));
        this.alliance = alliance;
    }

    public Alliance getAlliance() { return alliance; }

    public void setHiveEnd(HiveEnd end) {
        upEnd = end;
        MatchState.hiveEnd = end;
    }

    public void flipHiveEnd() {
        setHiveEnd(upEnd == HiveEnd.AUDIENCE ? HiveEnd.REAR : HiveEnd.AUDIENCE);
    }

    public HiveEnd getHiveEnd() { return upEnd; }

    /**
     * Pick the goal from the robot's side of the audience/rear center line (if AUTO_SWITCH_HIVE_END).
     * Has a small dead band around the line so it doesn't flicker while driving along it.
     */
    public void updateFromPose(double robotY) {
        if (!FieldConstants.AUTO_SWITCH_HIVE_END || Double.isNaN(robotY)) return;
        double line = FieldConstants.CENTER_LINE_Y;
        double band = FieldConstants.CENTER_LINE_HYSTERESIS_IN;
        if (robotY < line - band && upEnd != HiveEnd.AUDIENCE) setHiveEnd(HiveEnd.AUDIENCE);
        else if (robotY > line + band && upEnd != HiveEnd.REAR) setHiveEnd(HiveEnd.REAR);
    }

    public double getGoalX() { return FieldConstants.hiveCellX(alliance, upEnd); }
    public double getGoalY() { return FieldConstants.hiveCellY(alliance, upEnd); }

    /** Distance (in) from a field point to the target cell. */
    public double distanceFrom(double x, double y) {
        return Math.hypot(getGoalX() - x, getGoalY() - y);
    }
}
