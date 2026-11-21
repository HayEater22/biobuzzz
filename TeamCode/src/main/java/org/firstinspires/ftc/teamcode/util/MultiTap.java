package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

/** Detects N quick presses in a row (e.g. a triple tap). Call press() on each button press. */
public class MultiTap {

    private final int tapsRequired;
    private final double windowSec;
    private final ElapsedTime sinceLastTap = new ElapsedTime();
    private int count = 0;

    /** @param windowSec max time allowed between consecutive taps */
    public MultiTap(int tapsRequired, double windowSec) {
        this.tapsRequired = tapsRequired;
        this.windowSec = windowSec;
    }

    /** Register a press. Returns true on the press that completes the multi-tap. */
    public boolean press() {
        if (sinceLastTap.seconds() > windowSec) count = 0;
        sinceLastTap.reset();
        count++;
        if (count >= tapsRequired) {
            count = 0;
            return true;
        }
        return false;
    }
}
