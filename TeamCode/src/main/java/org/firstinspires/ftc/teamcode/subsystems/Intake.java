package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.HardwareNames;

/** Single-motor intake roller. */
@Config
public class Intake implements Subsystem {

    public static double INTAKE_POWER = 1.0;
    public static double REVERSE_POWER = -1.0;

    public enum State { IDLE, INTAKING, REVERSING }

    private final DcMotorEx motor;
    private State state = State.IDLE;

    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, HardwareNames.INTAKE);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void intake()  { state = State.INTAKING; }
    public void reverse() { state = State.REVERSING; }
    @Override public void stop() { state = State.IDLE; motor.setPower(0); }

    public State getState() { return state; }

    @Override
    public void periodic(Telemetry telemetry) {
        switch (state) {
            case INTAKING:  motor.setPower(INTAKE_POWER);  break;
            case REVERSING: motor.setPower(REVERSE_POWER); break;
            default:        motor.setPower(0);             break;
        }
        telemetry.addData("Intake", state);
    }
}
