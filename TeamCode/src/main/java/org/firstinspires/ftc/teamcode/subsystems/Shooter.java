package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.HardwareNames;

/**
 * Single-motor flywheel, controlled in RPM by a software loop (runs in periodic()):
 *
 *   power = (kS + kV * target) * (NOMINAL_VOLTAGE / battery) + kP * error + kI * integral(error)
 *
 * Feedforward (kS, kV) does most of the work and is battery-voltage compensated; kP fixes the rest.
 * Recovery boost: if RPM drops more than BOOST_THRESHOLD_RPM below target (a ball just went through),
 * the motor gets full power until it's within BOOST_EXIT_RPM of target, so the next ball leaves at
 * full speed. After a boost it won't fire again until the wheel has stayed within RPM_TOLERANCE for
 * BOOST_REARM_SEC, which stops it from toggling on/off (oscillating) while just holding speed.
 *
 * Spin-up ramp: the setpoint climbs toward the target at RAMP_RPM_PER_SEC (no boost while ramping),
 * so a hard full-power start doesn't make the belt skip.
 *
 * Tune live on FTC Dashboard ("Shooter" group) with the "Flywheel Tuner" OpMode.
 */
@Config
public class Shooter implements Subsystem {

    // ---- Tuning ----
    /** Encoder ticks per flywheel revolution (motor CPR x external gear ratio). 28 = bare goBILDA/REV motor. */
    public static double TICKS_PER_REV = 28.0;
    /** Static friction feedforward (power). */
    public static double kS = 0.00744;
    /** Power per RPM at NOMINAL_VOLTAGE. ~1/free-speed RPM to start (bare 6000 rpm motor -> ~0.00017). */
    public static double kV = 0.0001774;
    /** Power per RPM of error. Too high = RPM swings back and forth around the target. */
    public static double kP = 0.000560;
    /** Power per (RPM x second) of accumulated error. Slowly removes leftover error so kP can stay gentle. */
    public static double kI = 0.0000;
    /** Max power the kI term may add/remove. */
    public static double I_MAX_POWER = 0.2;
    /** Battery voltage kV was tuned at. */
    public static double NOMINAL_VOLTAGE = 12.5;
    public static boolean USE_BOOST = true;
    /**
     * Boost STARTS when RPM drops more than this far below target (lower = kicks in sooner).
     * Keep it above the normal wobble while holding speed, or the boost fires by itself and the RPM swings.
     */
    public static double BOOST_THRESHOLD_RPM = 100;
    /** Boost keeps full power until RPM is within this of target (lower = recovers harder). */
    public static double BOOST_EXIT_RPM = 25.0;
    /**
     * After a boost, the wheel must stay within RPM_TOLERANCE this long before the boost can fire again.
     * Keep it short: the next ball usually arrives ~0.1-0.2 s after a boost ends and needs its own boost.
     */
    public static double BOOST_REARM_SEC = 0.05;
    /** RPM low-pass filter, 0..1. 1 = no filtering, smaller = smoother but laggier. */
    public static double RPM_FILTER = 0.6;
    public static double RPM_TOLERANCE = 100.0;
    public static double IDLE_RPM = 0.0;
    /**
     * Spin-up ramp: the controller's setpoint climbs toward the target at most this fast, so the belt
     * doesn't skip. Lower = gentler. Speed DROPS (and stopping) are not ramped. Boost is off while ramping.
     */
    public static double RAMP_RPM_PER_SEC = 2000.0;
    /** Open-loop test mode only (auto tuner): max power increase per second, so the belt doesn't skip. */
    public static double OPEN_LOOP_SLEW_PER_SEC = 0.5;

    private static final double VOLTAGE_REFRESH_SEC = 0.1;

    private final DcMotorEx motor;
    private final VoltageSensor voltageSensor;
    private final ElapsedTime voltageTimer = new ElapsedTime();
    private double voltage = NOMINAL_VOLTAGE;
    private double targetRpm = 0.0;
    /** What the controller actually chases this loop: ramps up toward targetRpm. */
    private double setpointRpm = 0.0;
    private double filteredRpm = 0.0;
    private double rawRpm = 0.0;
    /** Test mode (auto tuner): send a fixed power, no controller. Any RPM command turns it off. */
    private boolean openLoop = false;
    private double openLoopPower = 0.0;
    private double power = 0.0;
    private boolean boosting = false;
    /** Boost may fire. Re-armed only once the wheel is back at speed, so it can't bang-bang while holding. */
    private boolean boostArmed = true;
    /** How long the wheel has been continuously within RPM_TOLERANCE (not boosting, not ramping). */
    private final ElapsedTime settledTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double integral = 0.0; // RPM x seconds

    public Shooter(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, HardwareNames.SHOOTER);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // encoder still counts; we do the control loop

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        voltage = voltageSensor.getVoltage();
    }

    /** Set flywheel speed target (dynamic input, e.g. from distance to goal). */
    public void setTargetRpm(double rpm) { openLoop = false; targetRpm = Math.max(0.0, rpm); }
    public void idle() { openLoop = false; targetRpm = IDLE_RPM; }

    /**
     * TESTING ONLY (auto tuner): bypass the controller and send this power (0..1), rising at most
     * OPEN_LOOP_SLEW_PER_SEC. setTargetRpm(), idle() or stop() return to normal control.
     */
    public void setOpenLoopPower(double p) {
        openLoop = true;
        targetRpm = 0.0;
        openLoopPower = Range.clip(p, 0.0, 1.0);
    }
    public boolean isOpenLoop() { return openLoop; }

    /** Clear the kI term's accumulated error (used by the auto tuner between trials). */
    public void resetIntegral() { integral = 0.0; }

    public double getTargetRpm() { return targetRpm; }
    /** Ramped setpoint the controller is chasing right now (equals the target once spin-up is done). */
    public double getSetpointRpm() { return setpointRpm; }
    public boolean isRamping() { return setpointRpm < targetRpm; }
    /** Filtered flywheel RPM. */
    public double getRpm() { return filteredRpm; }
    /** Unfiltered flywheel RPM this loop. */
    public double getRawRpm() { return rawRpm; }
    public double getPowerOutput() { return power; }
    public double getBatteryVoltage() { return voltage; }
    public boolean isBoosting() { return boosting; }

    public boolean isAtSpeed() {
        return targetRpm > 0 && Math.abs(filteredRpm - targetRpm) <= RPM_TOLERANCE;
    }

    @Override
    public void periodic(Telemetry telemetry) {
        // Voltage reads are slow (not in the bulk cache) - refresh only every 100 ms.
        if (voltageTimer.seconds() >= VOLTAGE_REFRESH_SEC) {
            double v = voltageSensor.getVoltage();
            if (v > 1.0) voltage = 0.7 * voltage + 0.3 * v;
            voltageTimer.reset();
        }

        double dt = Math.min(loopTimer.seconds(), 0.1);
        loopTimer.reset();

        rawRpm = motor.getVelocity() / TICKS_PER_REV * 60.0;
        double alpha = Range.clip(RPM_FILTER, 0.01, 1.0);
        filteredRpm += alpha * (rawRpm - filteredRpm);

        if (openLoop) {
            // Test mode: fixed power, slew-limited on the way up. Controller state stays reset.
            power = openLoopPower > power
                    ? Math.min(openLoopPower, power + Math.max(OPEN_LOOP_SLEW_PER_SEC, 0.01) * dt)
                    : openLoopPower;
            boosting = false;
            boostArmed = false;
            integral = 0.0;
            setpointRpm = Math.max(0.0, filteredRpm);
        } else if (targetRpm <= 0) {
            power = 0.0;
            boosting = false;
            boostArmed = false; // spin-up is ramped; boost only re-arms once at speed
            integral = 0.0;
            setpointRpm = Math.max(0.0, filteredRpm); // coasting: next spin-up ramps from the current speed
        } else {
            // Spin-up ramp: climb toward the target gradually; drops apply immediately.
            if (setpointRpm < targetRpm) {
                setpointRpm = Math.min(targetRpm, setpointRpm + Math.max(RAMP_RPM_PER_SEC, 1.0) * dt);
            } else {
                setpointRpm = targetRpm;
            }
            boolean ramping = setpointRpm < targetRpm;

            double error = setpointRpm - filteredRpm;
            // Hysteresis: enter boost far below target, stay in it until close to target.
            // Never boost during the spin-up ramp (full power is what skips the belt).
            if (boosting) {
                if (!USE_BOOST || ramping || error < BOOST_EXIT_RPM) {
                    boosting = false;
                    boostArmed = false; // don't boost again until we've settled at speed
                }
            } else if (USE_BOOST && !ramping && boostArmed && error > BOOST_THRESHOLD_RPM) {
                boosting = true;
            }
            // Re-arm only after the wheel has STAYED within tolerance for BOOST_REARM_SEC. (Checking just
            // "within tolerance" re-armed it in the same loop the boost ended, so it bang-banged.)
            boolean settledNow = !boosting && !ramping && Math.abs(error) <= RPM_TOLERANCE;
            if (!settledNow) settledTimer.reset();
            else if (settledTimer.seconds() >= BOOST_REARM_SEC) boostArmed = true;

            if (boosting) {
                power = 1.0;
            } else {
                double ff = (kS + kV * setpointRpm) * (NOMINAL_VOLTAGE / Math.max(voltage, 1.0));
                // Only integrate near target (not during spin-up / shot dips) so it doesn't wind up.
                if (Math.abs(error) <= BOOST_THRESHOLD_RPM) integral += error * dt;
                if (kI > 0) {
                    double iLimit = I_MAX_POWER / kI;
                    integral = Range.clip(integral, -iLimit, iLimit);
                }
                power = Range.clip(ff + kP * error + kI * integral, 0.0, 1.0); // never brake with reverse power
            }
        }
        motor.setPower(power);

        telemetry.addData("Shooter", "%.0f / %.0f rpm%s%s", filteredRpm, targetRpm,
                isAtSpeed() ? " READY" : "", openLoop ? String.format(" OPEN LOOP %.2f", power)
                        : boosting ? " BOOST" : isRamping() ? " RAMP" : "");
    }

    @Override
    public void stop() {
        openLoop = false;
        targetRpm = 0;
        power = 0;
        motor.setPower(0);
    }
}
