package org.firstinspires.ftc.teamcode.commands;

/** A unit of robot behavior that runs over multiple loops. */
public interface Command {
    /** Called once when the command starts. */
    default void initialize() {}

    /** Called every loop while the command is running. */
    void execute();

    /** Return true when the command is done. */
    boolean isFinished();

    /** Called once when the command ends (finished or cancelled). */
    default void end(boolean interrupted) {}
}
