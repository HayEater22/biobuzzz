package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Contract every mechanism follows: own its hardware, update once per loop. */
public interface Subsystem {
    /** Called exactly once per OpMode loop, after the bulk cache is cleared. */
    void periodic(Telemetry telemetry);

    /** Put the mechanism in a safe, idle state. */
    void stop();
}
