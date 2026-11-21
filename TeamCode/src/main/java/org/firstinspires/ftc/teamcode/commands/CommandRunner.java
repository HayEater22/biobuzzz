package org.firstinspires.ftc.teamcode.commands;

import java.util.ArrayList;
import java.util.List;

/** Minimal scheduler: runs any number of commands side by side. Call run() once per loop. */
public class CommandRunner {

    private final List<Command> running = new ArrayList<>();

    public void schedule(Command command) {
        if (running.contains(command)) return;
        command.initialize();
        running.add(command);
    }

    public boolean isRunning(Command command) { return running.contains(command); }

    public boolean isIdle() { return running.isEmpty(); }

    public void cancel(Command command) {
        if (running.remove(command)) command.end(true);
    }

    public void cancelAll() {
        for (Command c : running) c.end(true);
        running.clear();
    }

    public void run() {
        // Iterate a snapshot so commands may schedule/cancel other commands while running.
        for (Command c : new ArrayList<>(running)) {
            if (!running.contains(c)) continue; // cancelled earlier this loop
            c.execute();
            if (c.isFinished()) {
                c.end(false);
                running.remove(c);
            }
        }
    }
}
