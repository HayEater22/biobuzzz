package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.HardwareNames;
import org.firstinspires.ftc.teamcode.util.MathUtil;

/**
 * Servo-driven hood. Commanded as a fraction 0..1 of its travel
 * (0 = flattest shot, 1 = steepest), mapped onto the safe servo range.
 */
@Config
public class Hood implements Subsystem {

    // ---- Tuning ----
    /** Servo positions at the hood's mechanical extremes. Keep inside the hard stops. */
    public static double MIN_POSITION = 0.2;
    public static double MAX_POSITION = 0.8;
    /** Measured hood speed in travel-fractions per second, used to estimate arrival. */
    public static double SLEW_PER_SEC = 2.0;
    public static double AT_TARGET_TOLERANCE = 0.02;

    private final ServoImplEx servo;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double target = 0.0;
    private double estimated = 0.0;

    public Hood(HardwareMap hardwareMap) {
        servo = hardwareMap.get(ServoImplEx.class, HardwareNames.HOOD);
        servo.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    /** Set hood angle as a fraction 0..1 of its travel. */
    public void setAngle(double fraction) {
        target = Range.clip(fraction, 0.0, 1.0);
    }

    public double getTarget() { return target; }

    public boolean isAtTarget() { return Math.abs(estimated - target) <= AT_TARGET_TOLERANCE; }

    @Override
    public void periodic(Telemetry telemetry) {
        double dt = loopTimer.seconds();
        loopTimer.reset();
        estimated = MathUtil.approach(estimated, target, SLEW_PER_SEC * dt);

        double pos = MIN_POSITION + target * (MAX_POSITION - MIN_POSITION);
        servo.setPosition(pos);
        telemetry.addData("Hood", "%.2f (pos %.3f)", target, pos);
    }

    @Override
    public void stop() {}
}
