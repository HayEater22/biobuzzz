package org.firstinspires.ftc.teamcode.config;

/**
 * Values that carry over from Autonomous to TeleOp (static fields survive between OpModes
 * as long as the Robot Controller app stays running).
 */
public final class MatchState {
    private MatchState() {}

    public static FieldConstants.Alliance alliance = FieldConstants.Alliance.BLUE;

    /** Which end of our HIVE has the upward cell right now. */
    public static FieldConstants.HiveEnd hiveEnd = FieldConstants.startingHiveEnd(alliance);
}
