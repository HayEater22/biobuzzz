package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.config.HardwareNames;

import java.util.ArrayList;
import java.util.List;

/**
 * Test tool: spins one motor at a time by its config name and reports which hub port it is on
 * and how much current it draws. If power is sent but current stays ~0 A, nothing is plugged
 * into that port (or the config names the wrong port).
 * Motors missing from the config are listed as MISSING.
 */
public class MotorProbe {

    public static final class Entry {
        public final String name;
        final DcMotorEx motor;   // null if not in the config

        Entry(String name, DcMotorEx motor) {
            this.name = name;
            this.motor = motor;
        }

        public boolean isPresent() { return motor != null; }

        /** e.g. "Control Hub port 2" */
        public String getPort() {
            if (motor == null) return "MISSING from config";
            return motor.getConnectionInfo();
        }

        public double getCurrentAmps() {
            return motor == null ? 0.0 : motor.getCurrent(CurrentUnit.AMPS);
        }

        public double getPower() {
            return motor == null ? 0.0 : motor.getPower();
        }
    }

    private final List<Entry> entries = new ArrayList<>();
    private int selected = 0;

    public MotorProbe(HardwareMap hardwareMap) {
        add(hardwareMap, HardwareNames.INTAKE);
        add(hardwareMap, HardwareNames.TRANSFER);
        add(hardwareMap, HardwareNames.SHOOTER);
        add(hardwareMap, HardwareNames.FRONT_LEFT);
        add(hardwareMap, HardwareNames.FRONT_RIGHT);
        add(hardwareMap, HardwareNames.BACK_LEFT);
        add(hardwareMap, HardwareNames.BACK_RIGHT);
    }

    private void add(HardwareMap hardwareMap, String name) {
        DcMotorEx motor = hardwareMap.tryGet(DcMotorEx.class, name);
        if (motor != null) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        entries.add(new Entry(name, motor));
    }

    public List<Entry> getEntries() { return entries; }
    public Entry getSelected() { return entries.get(selected); }
    public int getSelectedIndex() { return selected; }

    /** Pick another motor; the previous one is stopped first. */
    public void selectNext(int delta) {
        stopAll();
        selected = Math.floorMod(selected + delta, entries.size());
    }

    /** Run only the selected motor at the given power (-1..1). */
    public void run(double power) {
        Entry e = getSelected();
        if (e.motor != null) e.motor.setPower(power);
    }

    public void stopAll() {
        for (Entry e : entries) {
            if (e.motor != null) e.motor.setPower(0);
        }
    }
}
