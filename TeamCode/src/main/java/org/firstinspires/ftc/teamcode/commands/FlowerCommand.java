package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.FlowerMech;

/**
 * Lowers the flower mech while running and raises it when the command ends.
 * TeleOp: whileHeld() — down while the button is held.
 * Auto:   timed(sec)  — down for a set time once the servos arrive, then back up.
 */
public class FlowerCommand implements Command {

    private final FlowerMech flower;
    private final double holdSec;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean arrived;

    private FlowerCommand(FlowerMech flower, double holdSec) {
        this.flower = flower;
        this.holdSec = holdSec;
    }

    public static FlowerCommand whileHeld(FlowerMech flower) {
        return new FlowerCommand(flower, Double.POSITIVE_INFINITY);
    }

    public static FlowerCommand timed(FlowerMech flower, double holdSec) {
        return new FlowerCommand(flower, holdSec);
    }

    @Override
    public void initialize() {
        flower.lower();
        arrived = false;
    }

    @Override
    public void execute() {
        if (!arrived && flower.isAtTarget()) {
            arrived = true;
            timer.reset();
        }
    }

    @Override
    public boolean isFinished() { return arrived && timer.seconds() >= holdSec; }

    @Override
    public void end(boolean interrupted) { flower.raise(); }
}
