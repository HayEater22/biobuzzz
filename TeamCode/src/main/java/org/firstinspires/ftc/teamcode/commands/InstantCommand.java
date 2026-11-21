package org.firstinspires.ftc.teamcode.commands;

/** Runs an action once and finishes immediately. */
public class InstantCommand implements Command {

    private final Runnable action;

    public InstantCommand(Runnable action) { this.action = action; }

    @Override public void initialize() { action.run(); }
    @Override public void execute() {}
    @Override public boolean isFinished() { return true; }
}
