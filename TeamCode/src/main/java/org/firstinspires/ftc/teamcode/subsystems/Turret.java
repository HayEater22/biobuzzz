package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.HardwareNames;
import org.firstinspires.ftc.teamcode.util.MathUtil;

/**
 * Servo-driven turret (two servos on one gear). Angle 0 = facing robot forward, positive = counter-clockwise
 * (to the robot's LEFT, seen from above).
 * Soft limits clamp every command so the turret can never wind up its wiring.
 *
 * Servo position = CENTER_POSITION + turretDeg * GEAR_RATIO / SERVO_TRAVEL_DEG  (sign flipped if INVERTED).
 * Calibrate GEAR_RATIO and INVERTED with the "Turret Calibration" OpMode.
 *
 * A servo reports no position, so the turret's actual angle is estimated by moving
 * toward the target at the servo's measured speed (SLEW_DEG_PER_SEC).
 */
@Config
public class Turret implements Subsystem {

    // ---- Tuning ----
    /** Servo position when the turret faces straight forward. */
    public static double CENTER_POSITION = 0.5;
    /** Servo rotation over its full 0..1 range. Servo spec: 0.175°/µs over 500-2500µs = 350°. */
    public static double SERVO_TRAVEL_DEG = 350.0;
    /** Servo degrees per turret degree (servo gear teeth : turret gear teeth inverted, e.g. 100T turret / 20T servo = 5). */
    public static double GEAR_RATIO = 1.0;
    /** true if a positive command turns the turret clockwise (to the right) instead of left. */
    public static boolean INVERTED = true;
    /** Software soft limits (turret degrees). Also limited by what the servo can physically reach. */
    public static double MIN_ANGLE_DEG = -175.0;
    public static double MAX_ANGLE_DEG = 175.0;
    /** Measured turret speed, used to estimate when it has arrived. */
    public static double SLEW_DEG_PER_SEC = 400.0;
    public static double AT_TARGET_TOLERANCE_DEG = 2.0;
    /** Set true if turret2 is mounted mirrored and must spin opposite to turret1 (applied at init). */
    public static boolean TURRET_2_REVERSED = false;
    /** Small position trim added to turret2 only, so the two servos don't fight each other. */
    public static double TURRET_2_TRIM = 0.0;

    private final ServoImplEx servo1;
    private final ServoImplEx servo2;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double targetDeg = 0.0;
    private double estimatedDeg = 0.0;
    private boolean clamped = false;

    public Turret(HardwareMap hardwareMap) {
        servo1 = hardwareMap.get(ServoImplEx.class, HardwareNames.TURRET_1);
        servo2 = hardwareMap.get(ServoImplEx.class, HardwareNames.TURRET_2);
        servo1.setPwmRange(new PwmControl.PwmRange(500, 2500));
        servo2.setPwmRange(new PwmControl.PwmRange(500, 2500));
        servo2.setDirection(TURRET_2_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
    }

    /** Furthest turret angle the servo can physically reach either side of center. */
    public static double reachDeg() {
        double servoHalf = Math.min(CENTER_POSITION, 1.0 - CENTER_POSITION) * SERVO_TRAVEL_DEG;
        return servoHalf / Math.abs(GEAR_RATIO);
    }

    /** Command a turret angle in degrees (robot frame). Out-of-range targets are clamped to the soft limits. */
    public void setAngle(double angleDeg) {
        if (Double.isNaN(angleDeg)) { clamped = true; return; }   // bad pose: hold the last target
        double min = Math.max(MIN_ANGLE_DEG, -reachDeg());
        double max = Math.min(MAX_ANGLE_DEG, reachDeg());
        targetDeg = Range.clip(angleDeg, min, max);
        clamped = targetDeg != angleDeg;
    }

    public void center() { setAngle(0.0); }

    public double getTargetAngle() { return targetDeg; }
    public double getEstimatedAngle() { return estimatedDeg; }

    /** True if the last requested angle was outside the soft limits (the goal is out of reach). */
    public boolean isAtLimit() { return clamped; }

    public boolean isAtTarget() { return Math.abs(estimatedDeg - targetDeg) <= AT_TARGET_TOLERANCE_DEG; }

    /** Servo position for a turret angle. */
    public static double positionFor(double turretDeg) {
        double sign = INVERTED ? -1.0 : 1.0;
        return Range.clip(CENTER_POSITION + sign * turretDeg * GEAR_RATIO / SERVO_TRAVEL_DEG, 0.0, 1.0);
    }

    @Override
    public void periodic(Telemetry telemetry) {
        double dt = loopTimer.seconds();
        loopTimer.reset();
        estimatedDeg = MathUtil.approach(estimatedDeg, targetDeg, SLEW_DEG_PER_SEC * dt);

        double pos = positionFor(targetDeg);
        servo1.setPosition(pos);
        servo2.setPosition(Range.clip(pos + TURRET_2_TRIM, 0.0, 1.0));
        telemetry.addData("Turret", "%.1f° (est %.1f°, servo %.3f)%s", targetDeg, estimatedDeg, pos,
                clamped ? " LIMIT" : "");
    }

    @Override
    public void stop() {}
}
