package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;

/** Does nothing for a fixed time. */
public class WaitCommand implements Command {

    private final double seconds;
    private final ElapsedTime timer = new ElapsedTime();

    public WaitCommand(double seconds) { this.seconds = seconds; }

    @Override public void initialize() { timer.reset(); }
    @Override public void execute() {}
    @Override public boolean isFinished() { return timer.seconds() >= seconds; }
}
