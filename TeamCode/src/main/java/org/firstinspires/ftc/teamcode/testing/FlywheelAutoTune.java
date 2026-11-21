package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Test tool: tunes the flywheel's existing controller (Shooter kS, kV, kP, kI, USE_BOOST,
 * BOOST_THRESHOLD_RPM) automatically. Drive it from an OpMode: call update() once per loop
 * AFTER robot.update(), and confirm() / skip() / abort() from buttons.
 *
 * Phases (each can be switched off on the Dashboard):
 *   1. Feedforward  open-loop power steps, least-squares fit of  power x V/V_nominal = kS + kV x rpm
 *   2. kP           closed-loop 300 RPM steps at several kP; largest kP that doesn't oscillate, x 0.7
 *   3. kI           hold at several kI; smallest kI that removes steady error without adding wobble
 *   4. Boost        real 4-ball bursts, one reload per setting; least RPM drop wins
 *
 * Each phase writes its winner into the Shooter statics (live). Nothing is saved to the code:
 * copy the final block shown on the Driver Station (also in the logcat, tag "FlywheelAutoTune").
 */
@Config
public class FlywheelAutoTune {

    // ---- Settings (Dashboard group "FlywheelAutoTune") ----
    /** RPM to tune at: use the RPM you actually shoot at. */
    public static double TUNE_RPM = 2450;
    public static boolean TUNE_FF = true;
    public static boolean TUNE_KP = true;
    public static boolean TUNE_KI = true;
    public static boolean TUNE_BOOST = true;
    /** Highest open-loop power used in the feedforward phase. */
    public static double MAX_POWER = 0.9;
    /** BOOST_THRESHOLD_RPM values to try (a "boost off" run is always added). */
    public static double[] BOOST_CANDIDATES = {50, 100, 150, 200, 300};
    /** How long to feed in each boost burst (long enough for all 4 balls). */
    public static double FEED_SEC = 2.5;

    private static final double[] KP_CANDIDATES = {0.00005, 0.0001, 0.0002, 0.0004, 0.0008};
    private static final double[] KI_CANDIDATES = {0.0, 0.0001, 0.0002, 0.0004};
    private static final double FF_MIN_POWER = 0.3;
    private static final int FF_STEPS = 6;
    private static final double STEP_DOWN_RPM = 300;
    private static final double MIN_R_SQUARED = 0.95;
    private static final String TAG = "FlywheelAutoTune";

    public enum Phase { READY, FEEDFORWARD, KP, KI, BOOST, DONE, ABORTED }

    private final Robot robot;
    private final Shooter shooter;

    private Phase phase = Phase.READY;
    private int trial = 0;
    private int stage = 0;
    private boolean confirmed = false;
    private String status = "Press A to start (B = stop, X = skip phase)";
    private String prompt = null;
    private final List<String> log = new ArrayList<>();
    private final List<String> results = new ArrayList<>();

    private final ElapsedTime stageTimer = new ElapsedTime();
    private final ElapsedTime subTimer = new ElapsedTime();
    private final ElapsedTime stallTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();

    // Values before tuning (for the old -> new table)
    private double origKS, origKV, origKP, origKI, origThreshold;
    private boolean origBoost;
    // Values at the start of the current phase (restored on skip / abort)
    private double snapKP, snapKI, snapThreshold;
    private boolean snapBoost;

    // Measurements
    private final List<double[]> ffPoints = new ArrayList<>();   // {rpm, normalized power}
    private double ffPower, checkpointRpm;
    private double[] trialMean, trialStd, trialOvershoot;
    private double[] boostSettings, boostDrop, boostBelowSec;
    private boolean[] boostDisqualified;
    private double maxRpm, minRpm, belowSec;
    private int boostEdges;
    private boolean lastBoosting;
    private double n, sum, sumSq, sumVolts;

    public FlywheelAutoTune(Robot robot) {
        this.robot = robot;
        this.shooter = robot.shooter;
    }

    // ================= Buttons =================

    /** A: start, or continue after a prompt. */
    public void confirm() {
        if (phase == Phase.READY) start();
        else if (prompt != null) confirmed = true;
    }

    /** X: skip the current phase, keeping the values from before it. */
    public void skip() {
        if (!isRunning()) return;
        restoreSnapshot();
        stopFeeding();
        addLog(phase + " skipped (old values kept)");
        nextPhase();
    }

    /** B / errors: stop everything. Values from finished phases stay; the current phase is undone. */
    public void abort(String reason) {
        if (!isRunning()) return;
        restoreSnapshot();
        stopFeeding();
        shooter.setTargetRpm(0);   // no power: the wheel coasts
        prompt = null;
        addLog("ABORTED in " + phase + ": " + reason);
        status = "Aborted: " + reason;
        phase = Phase.ABORTED;
        buildResults();
    }

    // ================= Getters for the OpMode =================

    public Phase getPhase() { return phase; }
    public boolean isRunning() { return phase != Phase.READY && phase != Phase.DONE && phase != Phase.ABORTED; }
    public String getStatus() { return status; }
    /** Non-null = waiting for the driver to do this, then press A. */
    public String getPrompt() { return prompt; }
    public List<String> getLog() { return log; }
    public List<String> getResults() { return results; }

    // ================= Loop =================

    public void update() {
        double dt = Math.min(loopTimer.seconds(), 0.1);
        loopTimer.reset();
        if (!isRunning() || safetyTripped()) return;

        switch (phase) {
            case FEEDFORWARD: updateFeedforward(); break;
            case KP:          updateKp(); break;
            case KI:          updateKi(); break;
            case BOOST:       updateBoost(dt); break;
            default: break;
        }
    }

    private boolean safetyTripped() {
        double rpm = shooter.getRpm();
        if (rpm < -50) {
            abort("RPM reads negative - motor or encoder direction is wrong");
            return true;
        }
        if (shooter.getPowerOutput() > 0.3 && rpm < 100) {
            if (stallTimer.seconds() > 1.5) {
                abort("power is on but RPM stays ~0 - check the encoder cable");
                return true;
            }
        } else {
            stallTimer.reset();
        }
        return false;
    }

    // ================= Phase sequencing =================

    private void start() {
        origKS = Shooter.kS;
        origKV = Shooter.kV;
        origKP = Shooter.kP;
        origKI = Shooter.kI;
        origBoost = Shooter.USE_BOOST;
        origThreshold = Shooter.BOOST_THRESHOLD_RPM;
        log.clear();
        results.clear();
        nextPhase();
    }

    private void nextPhase() {
        Phase[] order = {Phase.FEEDFORWARD, Phase.KP, Phase.KI, Phase.BOOST};
        boolean[] enabled = {TUNE_FF, TUNE_KP, TUNE_KI, TUNE_BOOST};
        int from = 0;
        for (int i = 0; i < order.length; i++) if (order[i] == phase) from = i + 1;
        for (int i = from; i < order.length; i++) {
            if (enabled[i]) { enterPhase(order[i]); return; }
        }
        finishAll();
    }

    private void enterPhase(Phase p) {
        phase = p;
        trial = 0;
        stage = 0;
        prompt = null;
        confirmed = false;
        snapKP = Shooter.kP;
        snapKI = Shooter.kI;
        snapBoost = Shooter.USE_BOOST;
        snapThreshold = Shooter.BOOST_THRESHOLD_RPM;
        switch (p) {
            case FEEDFORWARD:
                ffPoints.clear();
                break;
            case KP:
                Shooter.USE_BOOST = false;
                Shooter.kI = 0.0;
                newTrialArrays(KP_CANDIDATES.length);
                break;
            case KI:
                Shooter.USE_BOOST = false;
                newTrialArrays(KI_CANDIDATES.length);
                break;
            case BOOST:
                boostSettings = new double[BOOST_CANDIDATES.length + 1];
                boostSettings[0] = -1;   // boost off
                for (int i = 0; i < BOOST_CANDIDATES.length; i++) boostSettings[i + 1] = BOOST_CANDIDATES[i];
                boostDrop = new double[boostSettings.length];
                boostBelowSec = new double[boostSettings.length];
                boostDisqualified = new boolean[boostSettings.length];
                break;
            default: break;
        }
        addLog("== " + p + " ==");
    }

    private void restoreSnapshot() {
        Shooter.kP = snapKP;
        Shooter.kI = snapKI;
        Shooter.USE_BOOST = snapBoost;
        Shooter.BOOST_THRESHOLD_RPM = snapThreshold;
    }

    private void newTrialArrays(int count) {
        trialMean = new double[count];
        trialStd = new double[count];
        trialOvershoot = new double[count];
    }

    // ================= 1. Feedforward =================

    private void updateFeedforward() {
        double rpm = shooter.getRpm();
        switch (stage) {
            case 0: // command the next power step
                ffPower = FF_MIN_POWER + (MAX_POWER - FF_MIN_POWER) * trial / (FF_STEPS - 1.0);
                shooter.setOpenLoopPower(ffPower);
                checkpointRpm = rpm;
                stageTimer.reset();
                subTimer.reset();
                stage = 1;
                break;
            case 1: // wait for RPM to settle (< 20 RPM change over 0.5 s), max 6 s
                status = fmt("Feedforward: step %d/%d, power %.2f, settling (%.0f rpm)",
                        trial + 1, FF_STEPS, ffPower, rpm);
                if (subTimer.seconds() >= 0.5) {
                    boolean reached = shooter.getPowerOutput() >= ffPower - 1e-6;
                    boolean settled = reached && Math.abs(rpm - checkpointRpm) < 20;
                    checkpointRpm = rpm;
                    subTimer.reset();
                    if (settled || stageTimer.seconds() >= 6.0) {
                        resetStats();
                        stageTimer.reset();
                        stage = 2;
                    }
                }
                break;
            case 2: // average for 0.5 s
                addSample(shooter.getRawRpm());
                sumVolts += shooter.getBatteryVoltage();
                status = fmt("Feedforward: step %d/%d, measuring", trial + 1, FF_STEPS);
                if (stageTimer.seconds() >= 0.5) {
                    double avgRpm = mean();
                    double volts = sumVolts / n;
                    double normPower = ffPower * volts / Shooter.NOMINAL_VOLTAGE;
                    ffPoints.add(new double[]{avgRpm, normPower});
                    addLog(fmt("power %.2f @ %.2f V -> %.0f rpm", ffPower, volts, avgRpm));
                    trial++;
                    stage = 0;
                    if (trial >= FF_STEPS) finishFeedforward();
                }
                break;
            default: break;
        }
    }

    private void finishFeedforward() {
        double sx = 0, sy = 0, sxx = 0, sxy = 0, syy = 0, k = ffPoints.size();
        for (double[] pt : ffPoints) {
            sx += pt[0]; sy += pt[1];
            sxx += pt[0] * pt[0]; sxy += pt[0] * pt[1]; syy += pt[1] * pt[1];
        }
        double denomX = k * sxx - sx * sx;
        double denomY = k * syy - sy * sy;
        if (denomX <= 0 || denomY <= 0) {
            abort("feedforward data is flat - is the flywheel spinning?");
            return;
        }
        double kv = (k * sxy - sx * sy) / denomX;
        double ks = (sy - kv * sx) / k;
        double r = (k * sxy - sx * sy) / Math.sqrt(denomX * denomY);
        double r2 = r * r;
        if (kv <= 0 || r2 < MIN_R_SQUARED) {
            abort(fmt("feedforward fit is bad (kV %.6f, R^2 %.3f) - old values kept", kv, r2));
            return;
        }
        Shooter.kV = kv;
        Shooter.kS = Math.max(0.0, ks);
        addLog(fmt("-> kS %.4f  kV %.7f  (R^2 %.3f)", Shooter.kS, Shooter.kV, r2));
        nextPhase();
    }

    // ================= 2. kP =================

    private void updateKp() {
        double rpm = shooter.getRpm();
        double low = TUNE_RPM - STEP_DOWN_RPM;
        switch (stage) {
            case 0: // drop to the low speed
                Shooter.kP = KP_CANDIDATES[trial];
                shooter.resetIntegral();
                shooter.setTargetRpm(low);
                stageTimer.reset();
                stage = 1;
                break;
            case 1: // settle at low (at least 2.5 s, max 8 s)
                status = fmt("kP %d/%d (%.5f): settling at %.0f rpm", trial + 1, KP_CANDIDATES.length,
                        Shooter.kP, low);
                boolean near = Math.abs(rpm - low) <= Shooter.RPM_TOLERANCE;
                if ((stageTimer.seconds() >= 2.5 && near) || stageTimer.seconds() >= 8.0) {
                    shooter.setTargetRpm(TUNE_RPM);
                    resetStats();
                    maxRpm = rpm;
                    stageTimer.reset();
                    stage = 2;
                }
                break;
            case 2: // step up and watch for 3 s
                status = fmt("kP %d/%d (%.5f): step to %.0f rpm", trial + 1, KP_CANDIDATES.length,
                        Shooter.kP, TUNE_RPM);
                maxRpm = Math.max(maxRpm, rpm);
                if (stageTimer.seconds() >= 1.0) addSample(rpm);
                if (stageTimer.seconds() >= 3.0) {
                    trialOvershoot[trial] = maxRpm - TUNE_RPM;
                    trialMean[trial] = mean();
                    trialStd[trial] = std();
                    addLog(fmt("kP %.5f: overshoot %.0f, wobble +/-%.0f rpm",
                            Shooter.kP, trialOvershoot[trial], trialStd[trial]));
                    trial++;
                    stage = 0;
                    if (trial >= KP_CANDIDATES.length) finishKp();
                }
                break;
            default: break;
        }
    }

    private void finishKp() {
        double base = trialStd[0];
        int best = -1;
        for (int i = 0; i < KP_CANDIDATES.length; i++) {
            boolean oscillating = trialStd[i] > 1.5 * base + 5.0 || trialOvershoot[i] > 150.0;
            if (oscillating) break;          // larger kP only gets worse
            best = i;
        }
        double chosen = best >= 0 ? KP_CANDIDATES[best] * 0.7 : KP_CANDIDATES[0];
        Shooter.kP = chosen;
        Shooter.kI = snapKI;
        Shooter.USE_BOOST = snapBoost;
        addLog(fmt("-> kP %.6f", chosen));
        nextPhase();
    }

    // ================= 3. kI =================

    private void updateKi() {
        double rpm = shooter.getRpm();
        switch (stage) {
            case 0:
                Shooter.kI = KI_CANDIDATES[trial];
                shooter.resetIntegral();
                shooter.setTargetRpm(TUNE_RPM);
                stageTimer.reset();
                stage = 1;
                break;
            case 1: // reach the target (at least 0.5 s, max 6 s)
                status = fmt("kI %d/%d (%.5f): reaching %.0f rpm", trial + 1, KI_CANDIDATES.length,
                        Shooter.kI, TUNE_RPM);
                boolean near = Math.abs(rpm - TUNE_RPM) <= Shooter.RPM_TOLERANCE;
                if ((stageTimer.seconds() >= 0.5 && near) || stageTimer.seconds() >= 6.0) {
                    resetStats();
                    stageTimer.reset();
                    stage = 2;
                }
                break;
            case 2: // hold 4 s, measure the last 2 s
                status = fmt("kI %d/%d (%.5f): holding", trial + 1, KI_CANDIDATES.length, Shooter.kI);
                if (stageTimer.seconds() >= 2.0) addSample(TUNE_RPM - rpm);
                if (stageTimer.seconds() >= 4.0) {
                    trialMean[trial] = mean();
                    trialStd[trial] = std();
                    addLog(fmt("kI %.5f: avg error %.0f, wobble +/-%.0f rpm",
                            Shooter.kI, trialMean[trial], trialStd[trial]));
                    trial++;
                    stage = 0;
                    if (trial >= KI_CANDIDATES.length) finishKi();
                }
                break;
            default: break;
        }
    }

    private void finishKi() {
        double base = trialStd[0];
        double chosen = 0.0;
        for (int i = 0; i < KI_CANDIDATES.length; i++) {
            if (Math.abs(trialMean[i]) <= 15.0 && trialStd[i] <= 1.3 * base + 5.0) {
                chosen = KI_CANDIDATES[i];
                break;
            }
        }
        Shooter.kI = chosen;
        Shooter.USE_BOOST = snapBoost;
        shooter.resetIntegral();
        addLog(fmt("-> kI %.5f", chosen));
        nextPhase();
    }

    // ================= 4. Boost (real balls) =================

    private void updateBoost(double dt) {
        double rpm = shooter.getRpm();
        double setting = boostSettings[trial];
        String name = setting < 0 ? "boost OFF" : fmt("threshold %.0f", setting);
        String label = fmt("Boost %d/%d (%s)", trial + 1, boostSettings.length, name);
        switch (stage) {
            case 0: // coast, wait for balls
                shooter.setTargetRpm(0);
                stopFeeding();
                prompt = "Load 4 balls, then press A";
                status = label + ": waiting for balls";
                if (confirmed) {
                    confirmed = false;
                    prompt = null;
                    Shooter.USE_BOOST = setting >= 0;
                    if (setting >= 0) Shooter.BOOST_THRESHOLD_RPM = setting;
                    shooter.setTargetRpm(TUNE_RPM);
                    stageTimer.reset();
                    subTimer.reset();
                    stage = 1;
                }
                break;
            case 1: // spin up, then 1 s steady at speed (max 8 s)
                status = label + ": spinning up";
                if (!shooter.isAtSpeed()) subTimer.reset();
                if (subTimer.seconds() >= 1.0 || stageTimer.seconds() >= 8.0) {
                    boostEdges = 0;
                    lastBoosting = shooter.isBoosting();
                    stageTimer.reset();
                    stage = 2;
                }
                break;
            case 2: // hold 2 s: boost must NOT fire while just holding speed
                status = label + ": holding (no balls yet)";
                if (shooter.isBoosting() && !lastBoosting) boostEdges++;
                lastBoosting = shooter.isBoosting();
                if (stageTimer.seconds() >= 2.0) {
                    robot.transfer.openGate();
                    robot.transfer.feed();
                    robot.intake.intake();
                    minRpm = rpm;
                    belowSec = 0;
                    stageTimer.reset();
                    stage = 3;
                }
                break;
            case 3: // feed the burst
                status = label + ": FIRING";
                trackBurst(rpm, dt);
                if (stageTimer.seconds() >= FEED_SEC) {
                    stopFeeding();
                    stageTimer.reset();
                    stage = 4;
                }
                break;
            case 4: // 2 s to recover
                status = label + ": recovering";
                trackBurst(rpm, dt);
                if (stageTimer.seconds() >= 2.0) {
                    boostDrop[trial] = TUNE_RPM - minRpm;
                    boostBelowSec[trial] = belowSec;
                    boostDisqualified[trial] = setting >= 0 && boostEdges > 0;
                    addLog(fmt("%s: drop %.0f rpm, %.2f s below speed%s", name, boostDrop[trial],
                            belowSec, boostDisqualified[trial] ? ", BOOSTED WHILE HOLDING (rejected)" : ""));
                    trial++;
                    stage = 0;
                    if (trial >= boostSettings.length) finishBoost();
                }
                break;
            default: break;
        }
    }

    private void trackBurst(double rpm, double dt) {
        minRpm = Math.min(minRpm, rpm);
        if (rpm < TUNE_RPM - Shooter.RPM_TOLERANCE) belowSec += dt;
    }

    private void finishBoost() {
        int best = -1;
        for (int i = 0; i < boostSettings.length; i++) {
            if (boostDisqualified[i]) continue;
            if (best < 0
                    || boostDrop[i] < boostDrop[best] - 30.0
                    || (Math.abs(boostDrop[i] - boostDrop[best]) <= 30.0 && boostBelowSec[i] < boostBelowSec[best])) {
                best = i;
            }
        }
        double setting = boostSettings[best];   // boost OFF can't be disqualified, so best >= 0
        Shooter.USE_BOOST = setting >= 0;
        if (setting >= 0) Shooter.BOOST_THRESHOLD_RPM = setting;
        addLog(setting < 0 ? "-> boost OFF" : fmt("-> boost ON, threshold %.0f", setting));
        shooter.setTargetRpm(0);
        nextPhase();
    }

    // ================= Results =================

    private void finishAll() {
        phase = Phase.DONE;
        prompt = null;
        stopFeeding();
        shooter.setTargetRpm(0);
        status = "Done - copy the values below into Shooter.java";
        buildResults();
    }

    private void buildResults() {
        results.clear();
        results.add(fmt("kS  %.4f -> %.4f", origKS, Shooter.kS));
        results.add(fmt("kV  %.7f -> %.7f", origKV, Shooter.kV));
        results.add(fmt("kP  %.6f -> %.6f", origKP, Shooter.kP));
        results.add(fmt("kI  %.5f -> %.5f", origKI, Shooter.kI));
        results.add(fmt("USE_BOOST  %b -> %b", origBoost, Shooter.USE_BOOST));
        results.add(fmt("BOOST_THRESHOLD_RPM  %.0f -> %.0f", origThreshold, Shooter.BOOST_THRESHOLD_RPM));

        String block = fmt("public static double kS = %.4f;%n", Shooter.kS)
                + fmt("public static double kV = %.7f;%n", Shooter.kV)
                + fmt("public static double kP = %.6f;%n", Shooter.kP)
                + fmt("public static double kI = %.5f;%n", Shooter.kI)
                + fmt("public static boolean USE_BOOST = %b;%n", Shooter.USE_BOOST)
                + fmt("public static double BOOST_THRESHOLD_RPM = %.1f;", Shooter.BOOST_THRESHOLD_RPM);
        RobotLog.ii(TAG, "Result (" + phase + "):\n" + block);
        for (String line : log) RobotLog.ii(TAG, line);
    }

    // ================= Helpers =================

    private void stopFeeding() {
        robot.transfer.closeGate();
        robot.transfer.stopMotor();
        robot.intake.stop();
    }

    private void resetStats() { n = 0; sum = 0; sumSq = 0; sumVolts = 0; }
    private void addSample(double x) { n++; sum += x; sumSq += x * x; }
    private double mean() { return n > 0 ? sum / n : 0; }
    private double std() {
        if (n < 2) return 0;
        double m = mean();
        return Math.sqrt(Math.max(0, sumSq / n - m * m));
    }

    private void addLog(String line) {
        log.add(line);
        RobotLog.ii(TAG, line);
    }

    private static String fmt(String f, Object... args) { return String.format(Locale.US, f, args); }
}
