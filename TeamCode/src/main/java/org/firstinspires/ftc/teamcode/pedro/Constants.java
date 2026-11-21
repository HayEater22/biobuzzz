package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.HardwareNames;

/**
 * Pedro Pathing configuration. Run AutoTune (see Tuning.java) in this order and paste
 * each generated block over the matching config below:
 *   1. Mecanum Tuner   -> drivetrainConfig (motor directions)
 *   2. Pinpoint Tuner  -> localizerConfig (pod directions + offsets)
 *   3. Foresight Tuner -> foresightConfig
 * Keep the names pointing at HardwareNames so the whole robot uses one set of names.
 */
public class Constants {

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set(HardwareNames.FRONT_LEFT);
        c.frontRightName.set(HardwareNames.FRONT_RIGHT);
        c.backLeftName.set(HardwareNames.BACK_LEFT);
        c.backRightName.set(HardwareNames.BACK_RIGHT);
        // TUNE with Mecanum Tuner
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.manualBrakeMode.set(false); // coast (float) when sticks are released in TeleOp
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set(HardwareNames.PINPOINT);
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        // Measured from robot center (mm). xPodOffset: + = left of center. yPodOffset: + = forward of center.
        c.xPodOffset.set(-83.0);     // forward pod: 83 mm RIGHT of center
        c.yPodOffset.set(-82.595);   // strafe pod: 82.595 mm BEHIND center
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.MM);
        // Only recalibrate the gyro on init; keep the pose so it carries from Auto into TeleOp.
        c.resetMode.set(PinpointLocalizer.ResetMode.RECALIBRATE_IMU);
    });

    /**
     * PLACEHOLDER VALUES copied from the Pedro docs example robot. They are NOT tuned for this
     * robot — run the Foresight Tuner before following any path.
     */
    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        Controller primaryTranslationalForward = Controller.proportional(0.3);
        Controller secondaryTranslationalForward = Controller.proportional(0.1);
        Controller primaryTranslationalLateral = Controller.proportional(0.3);
        Controller secondaryTranslationalLateral = Controller.proportional(0.1);

        c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
        c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

        c.coast.set(Controller.proportionalFeedforward(0.011));
        c.brake.set(Controller.proportionalFeedforward(0.0087));

        c.headingFeedback.set(Controller.proportional(5.26));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.056, 0.0064));

        c.linearBrakeCoefficients.set(Matrix.diag(0.106, 0.087));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.00147, 0.00138));

        c.maxAchievableForwardVelocity.set(72.7);
        c.maxAchievableStrafeVelocity.set(52.3);
        c.naturalForwardDeceleration.set(85.0);
        c.naturalStrafeDeceleration.set(104.5);
    });

    public static PinpointLocalizer createLocalizer(HardwareMap h) {
        return new PinpointLocalizer(h, localizerConfig);
    }

    public static Mecanum createDrivetrain(HardwareMap h) {
        return new Mecanum(h, drivetrainConfig);
    }

    public static Foresight createAlgorithm() {
        return new Foresight(foresightConfig);
    }

    public static Follower create(HardwareMap h) {
        return new Follower(createLocalizer(h), createDrivetrain(h), createAlgorithm());
    }
}
