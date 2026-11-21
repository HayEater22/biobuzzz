package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.HardwareNames;

/** Transfer motor that moves pieces toward the flywheel, plus a servo gate that holds them back. */
@Config
public class Transfer implements Subsystem {

    // ---- Tuning ----
    public static double GATE_CLOSED_POSITION = 1.0;
    public static double GATE_OPEN_POSITION = 0.7;
    public static double FEED_POWER = 1.0;
    public static double REVERSE_POWER = -1.0;

    public enum MotorState { STOPPED, FEEDING, REVERSING }

    private final DcMotorEx motor;
    private final Servo gate;
    private MotorState motorState = MotorState.STOPPED;
    private boolean gateOpen = false;

    public Transfer(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, HardwareNames.TRANSFER);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        gate = hardwareMap.get(Servo.class, HardwareNames.TRANSFER_GATE);
    }

    public void openGate()  { gateOpen = true; }
    public void closeGate() { gateOpen = false; }
    public boolean isGateOpen() { return gateOpen; }

    public void feed()      { motorState = MotorState.FEEDING; }
    public void reverse() { motorState = MotorState.REVERSING; }
    public void stopMotor() { motorState = MotorState.STOPPED; }

    @Override
    public void periodic(Telemetry telemetry) {
        gate.setPosition(gateOpen ? GATE_OPEN_POSITION : GATE_CLOSED_POSITION);
        switch (motorState) {
            case FEEDING:   motor.setPower(FEED_POWER);    break;
            case REVERSING: motor.setPower(REVERSE_POWER); break;
            default:        motor.setPower(0);             break;
        }
        telemetry.addData("Transfer", "%s, gate %s", motorState, gateOpen ? "OPEN" : "closed");
    }

    @Override
    public void stop() {
        closeGate();
        stopMotor();
        motor.setPower(0);
    }
}
