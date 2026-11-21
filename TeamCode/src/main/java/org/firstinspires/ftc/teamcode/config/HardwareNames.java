package org.firstinspires.ftc.teamcode.config;

/**
 * Every hardware configuration name used by the robot, in one place.
 * These MUST match the names in the Driver Station robot configuration.
 *
 * Robot hardware:
 *   Drivetrain:   4x DcMotorEx (mecanum)
 *   Localization: goBILDA Pinpoint (I2C)
 *   Intake:       1x DcMotorEx
 *   Transfer:     1x DcMotorEx + 1x Servo (gate)
 *   Turret:       2x Servo (both drive the same turret, commanded together)
 *   Shooter:      1x DcMotorEx (flywheel, with encoder)
 *   Hood:         1x Servo
 *   Flower mech:  2x Servo (mounted facing each other)
 */
public final class HardwareNames {
    private HardwareNames() {}

    // Drivetrain
    public static final String FRONT_LEFT  = "fl";
    public static final String FRONT_RIGHT = "fr";
    public static final String BACK_LEFT   = "bl";
    public static final String BACK_RIGHT  = "br";

    // Localization
    public static final String PINPOINT    = "pinpoint";

    // Mechanisms
    public static final String INTAKE        = "intake";
    public static final String TRANSFER      = "transfer";
    public static final String TRANSFER_GATE = "gate";
    public static final String TURRET_1      = "turret1";
    public static final String TURRET_2      = "turret2";
    public static final String SHOOTER       = "shooter";
    public static final String HOOD          = "hood";
    public static final String FLOWER_LEFT   = "flowerLeft";
    public static final String FLOWER_RIGHT  = "flowerRight";
}
