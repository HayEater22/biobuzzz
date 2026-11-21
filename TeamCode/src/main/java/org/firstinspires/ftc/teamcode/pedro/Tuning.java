package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;

/**
 * Pedro AutoTune procedures. Connect to the robot's Wi-Fi and open
 *   Control Hub: http://192.168.43.1:10158
 * Run them in order: Mecanum -> Pinpoint -> Foresight, pasting each result into Constants.
 */
public class Tuning {

    @Tuner
    public static Procedure mecanum() {
        return new MecanumTuner();
    }

    @Tuner
    public static Procedure pinpoint() {
        return new PinpointTuner();
    }

    @Tuner
    public static Procedure foresight() {
        return new ForesightTuner(Constants::createLocalizer, Constants::createDrivetrain);
    }

    @Tuner
    public static Procedure tests() {
        return new Tests(Constants::createDrivetrain, Constants::createLocalizer, Constants::createAlgorithm);
    }
}
