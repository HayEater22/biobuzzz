package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.FieldConstants;
import org.firstinspires.ftc.teamcode.config.FieldConstants.Alliance;
import org.firstinspires.ftc.teamcode.config.MatchState;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.FlowerMech;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Localization;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.targeting.GoalSelector;
import org.firstinspires.ftc.teamcode.util.BulkCache;

/** Owns every subsystem and the hub bulk cache. OpModes create one Robot and call update() each loop. */
public class Robot {

    public final Localization localization;
    public final Drivetrain drivetrain;
    public final Intake intake;
    public final Transfer transfer;
    public final Turret turret;
    public final Shooter shooter;
    public final Hood hood;
    /** Flower mech, or null when the OpMode leaves it out (its servo is then never looked up or commanded). */
    public final FlowerMech flower;
    public final GoalSelector goal;

    private final BulkCache bulkCache;
    private final Subsystem[] subsystems;

    public Robot(HardwareMap hardwareMap) {
        this(hardwareMap, true);
    }

    /** @param withFlower false = don't create the flower mech at all: its servo is never looked up or sent anything. */
    public Robot(HardwareMap hardwareMap, boolean withFlower) {
        bulkCache = new BulkCache(hardwareMap);

        drivetrain   = new Drivetrain(hardwareMap);          // Pedro Follower: drive motors + Pinpoint
        localization = new Localization(drivetrain);
        intake       = new Intake(hardwareMap);
        transfer     = new Transfer(hardwareMap);
        turret       = new Turret(hardwareMap);
        shooter      = new Shooter(hardwareMap);
        hood         = new Hood(hardwareMap);
        flower       = withFlower ? new FlowerMech(hardwareMap) : null;
        goal         = new GoalSelector(MatchState.alliance);

        // Drivetrain first: its follower.update() reads the Pinpoint, so everything after sees this loop's pose.
        subsystems = withFlower
                ? new Subsystem[]{drivetrain, localization, intake, transfer, turret, shooter, hood, flower}
                : new Subsystem[]{drivetrain, localization, intake, transfer, turret, shooter, hood};
        setAlliance(MatchState.alliance);
    }

    /** Select alliance: sets the target goal and the driver's field-oriented "forward". Remembered into TeleOp. */
    public void setAlliance(Alliance alliance) {
        MatchState.alliance = alliance;
        goal.setAlliance(alliance);
        drivetrain.setDriverForwardHeading(FieldConstants.driverHeadingDeg(alliance));
    }

    public Alliance getAlliance() { return goal.getAlliance(); }

    /**
     * Snap the field pose back to the alliance's known relocalize spot
     * (robot must be physically squared up there) and restore the driver's forward direction.
     */
    public void relocalize() {
        Alliance a = getAlliance();
        localization.setPose(FieldConstants.relocalizeX(a), FieldConstants.relocalizeY(a),
                FieldConstants.relocalizeHeadingDeg(a));
        drivetrain.setDriverForwardHeading(FieldConstants.driverHeadingDeg(a));
    }

    /** Call once at the TOP of every loop: clears the bulk cache, then runs every subsystem's periodic(). */
    public void update(Telemetry telemetry) {
        bulkCache.clear();
        for (Subsystem s : subsystems) {
            s.periodic(telemetry);
        }
    }

    public void stopAll() {
        for (Subsystem s : subsystems) {
            s.stop();
        }
    }
}
