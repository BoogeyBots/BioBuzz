package org.firstinspires.ftc.teamcode.Module;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeModule {
    private final HardwareMap hardwareMap;
    private DcMotorEx mi, mtr;
    private Servo servo_faras;

    private double lastIntakePower = 0.0;
    private double lastTransferPower = 0.0;
    private double lastFarasPosition = -1.0;

    private static final double POWER_THRESHOLD = 0.01;

    public IntakeModule(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        mi = hardwareMap.get(DcMotorEx.class, "motor_intake");
        mtr = hardwareMap.get(DcMotorEx.class, "motor_transfer");
        servo_faras = hardwareMap.get(Servo.class, "servo_faras");

        mi.setDirection(DcMotorSimple.Direction.REVERSE);
        setFarasPosition(0.82);
    }

    public void setIntakePower(double power) {
        if (Math.abs(power) < POWER_THRESHOLD) {
            power = 0.0;
        }

        if (Math.abs(power - lastIntakePower) > POWER_THRESHOLD || (power == 0.0 && lastIntakePower != 0.0)) {
            mi.setPower(power);
            lastIntakePower = power;
        }
    }

    public void setTransferPower(double power) {
        if (Math.abs(power) < POWER_THRESHOLD) {
            power = 0.0;
        }

        if (Math.abs(power - lastTransferPower) > POWER_THRESHOLD || (power == 0.0 && lastTransferPower != 0.0)) {
            mtr.setPower(power);
            lastTransferPower = power;
        }
    }

    public void trage_intake(double power) {
        setIntakePower(Math.abs(power));
    }

    public void scuipa_intake(double power) {
        setIntakePower(-Math.abs(power));
    }

    public void stop_intake() {
        setIntakePower(0.0);
    }

    public void trage_transfer(double power) {
        setTransferPower(Math.abs(power));
    }

    public void scuipa_transfer(double power) {
        setTransferPower(-Math.abs(power));
    }

    public void stop_transfer() {
        setTransferPower(0.0);
    }

    public void setFarasPosition(double targetPosition) {
        if (Math.abs(targetPosition - lastFarasPosition) > 0.005) {
            servo_faras.setPosition(targetPosition);
            lastFarasPosition = targetPosition;
        }
    }

    public void farassus() {
        setFarasPosition(0.81);
    }

    public void farasjos() {
        setFarasPosition(0.29);
    }
}
