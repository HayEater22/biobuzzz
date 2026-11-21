package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.HardwareNames;

/**
 * Flower mech, driven by the RIGHT servo only.
 *
 * The left servo is dead: it is never looked up, initialized or sent any signal.
 */
@Config
public class FlowerMech implements Subsystem {

    // ---- Tuning ----
    public static double RAISED_POSITION = 0.5;
    public static double LOWERED_POSITION = 0.0;
    /** Time for the servo to physically finish a move. */
    public static double MOVE_TIME_SEC = 0.3;

    private final ServoImplEx right;
    private final ElapsedTime moveTimer = new ElapsedTime();
    private boolean lowered = false;

    public FlowerMech(HardwareMap hardwareMap) {
        right = hardwareMap.get(ServoImplEx.class, HardwareNames.FLOWER_RIGHT);
        right.setPwmRange(new PwmControl.PwmRange(500, 2500));
        right.setDirection(Servo.Direction.REVERSE);   // same setup the servo test tools use
    }

    public void lower() {
        if (!lowered) moveTimer.reset();
        lowered = true;
    }

    public void raise() {
        if (lowered) moveTimer.reset();
        lowered = false;
    }

    public boolean isLowered() { return lowered; }

    /** True once the servo has had time to finish its last move. */
    public boolean isAtTarget() { return moveTimer.seconds() >= MOVE_TIME_SEC; }

    @Override
    public void periodic(Telemetry telemetry) {
        right.setPosition(lowered ? LOWERED_POSITION : RAISED_POSITION);
        telemetry.addData("Flower", lowered ? "LOWERED" : "raised");
    }

    @Override
    public void stop() { raise(); }
}
