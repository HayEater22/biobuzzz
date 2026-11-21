package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.config.HardwareNames;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.ArrayList;
import java.util.List;

/**
 * Test tool: moves one robot servo at a time so you can find its positions.
 * Uses the same setup as the real subsystems (500-2500us range, flowerRight reversed),
 * and moves both turret servos together (with turret2's reverse + trim) since they share one gear.
 * so the numbers you find can be copied straight into the code.
 * Servos missing from the config are skipped.
 */
public class ServoProbe {

    public static final class Entry {
        public final String name;
        final List<ServoImplEx> servos = new ArrayList<>();
        final List<Double> trims = new ArrayList<>();   // per-servo position offset (turret2 trim)
        double position = 0.5;
        boolean moved = false;

        Entry(String name) {
            this.name = name;
        }

        public double getPosition() { return position; }

        /** Hub + port this entry's servo(s) are on, e.g. "Control Hub servo port 2". */
        public String getPort() {
            StringBuilder sb = new StringBuilder();
            for (ServoImplEx s : servos) {
                if (sb.length() > 0) sb.append(" + ");
                sb.append(s.getConnectionInfo());
            }
            return sb.toString();
        }
        public boolean hasMoved() { return moved; }
    }

    private final List<Entry> entries = new ArrayList<>();
    private int selected = 0;

    public ServoProbe(HardwareMap hardwareMap) {
        addTurret(hardwareMap);
        // Each turret servo alone: for finding its port / checking direction. GEAR OFF ONLY.
        add(hardwareMap, HardwareNames.TURRET_1, true, false, "turret1 ALONE");
        add(hardwareMap, HardwareNames.TURRET_2, true, Turret.TURRET_2_REVERSED, "turret2 ALONE");
        add(hardwareMap, HardwareNames.HOOD, true, false);
        add(hardwareMap, HardwareNames.TRANSFER_GATE, false, false);   // gate keeps the default range
        add(hardwareMap, HardwareNames.FLOWER_LEFT, true, false);
        add(hardwareMap, HardwareNames.FLOWER_RIGHT, true, true);      // mirrored, same as FlowerMech
    }

    private void add(HardwareMap hardwareMap, String name, boolean wideRange, boolean reversed) {
        add(hardwareMap, name, wideRange, reversed, name);
    }

    private void add(HardwareMap hardwareMap, String name, boolean wideRange, boolean reversed, String label) {
        ServoImplEx servo = setUp(hardwareMap, name, wideRange, reversed);
        if (servo == null) return;
        Entry e = new Entry(label);
        e.servos.add(servo);
        e.trims.add(0.0);
        entries.add(e);
    }

    /** Both turret servos as one entry, so they never fight each other. */
    private void addTurret(HardwareMap hardwareMap) {
        Entry e = new Entry("turret (1+2)");
        ServoImplEx s1 = setUp(hardwareMap, HardwareNames.TURRET_1, true, false);
        ServoImplEx s2 = setUp(hardwareMap, HardwareNames.TURRET_2, true, Turret.TURRET_2_REVERSED);
        if (s1 != null) { e.servos.add(s1); e.trims.add(0.0); }
        if (s2 != null) { e.servos.add(s2); e.trims.add(Turret.TURRET_2_TRIM); }
        if (!e.servos.isEmpty()) entries.add(e);
    }

    private static ServoImplEx setUp(HardwareMap hardwareMap, String name, boolean wideRange, boolean reversed) {
        Servo s = hardwareMap.tryGet(Servo.class, name);
        if (!(s instanceof ServoImplEx)) return null;
        ServoImplEx servo = (ServoImplEx) s;
        if (wideRange) servo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        servo.setDirection(reversed ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        return servo;
    }

    public List<Entry> getEntries() { return entries; }
    public boolean isEmpty() { return entries.isEmpty(); }
    public Entry getSelected() { return entries.get(selected); }
    public int getSelectedIndex() { return selected; }

    public void selectNext(int delta) {
        if (entries.isEmpty()) return;
        selected = Math.floorMod(selected + delta, entries.size());
    }

    /** Move the selected servo by delta (0..1 scale). The first press moves it to 0.5 +/- delta. */
    public void nudge(double delta) {
        if (entries.isEmpty()) return;
        Entry e = getSelected();
        e.position = Range.clip(e.position + delta, 0.0, 1.0);
        e.moved = true;
        for (int i = 0; i < e.servos.size(); i++) {
            e.servos.get(i).setPosition(Range.clip(e.position + e.trims.get(i), 0.0, 1.0));
        }
    }

    /** Turn off the selected servo's signal so it goes limp. Next nudge turns it back on. */
    public void release() {
        if (entries.isEmpty()) return;
        Entry e = getSelected();
        for (ServoImplEx s : e.servos) s.setPwmDisable();
        e.moved = false;
    }
}
