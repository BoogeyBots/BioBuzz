package org.firstinspires.ftc.teamcode.Module;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class IntakeModule {

    HardwareMap hardwareMap;

    public IntakeModule(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    DcMotorEx mi, mtr;
    Servo servo_faras;

    private double lastIntakePower = 0;
    private double lastTransferPower = 0;


    public void init(){
      //  mi = hardwareMap.get(DcMotorEx.class, "motor_intake");
        //mtr = hardwareMap.get(DcMotorEx.class, "motor_transfer");
        servo_faras = hardwareMap.get(Servo.class, "servo_faras");

        servo_faras.setPosition(0.82);
        //mi.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void trage_intake(double power) {
        double target = power;
        if (Math.abs(target - lastIntakePower) > 0.005) {
            mi.setPower(target);
            lastIntakePower = target;
        }
    }

    public void scuipa_intake(double power) {
        double target = -power;
        if (Math.abs(target - lastIntakePower) > 0.005) {
            mi.setPower(target);
            lastIntakePower = target;
        }
    }

    public void stop_intake() {
        if (lastIntakePower != 0) {
            mi.setPower(0);
            lastIntakePower = 0;
        }
    }
    public void trage_transfer(double power) {
        double target = power;
        if (Math.abs(target - lastTransferPower) > 0.005) {
            mtr.setPower(target);
            lastTransferPower = target;
        }
    }

    public void scuipa_transfer(double power) {
        double target = -power;
        if (Math.abs(target - lastTransferPower) > 0.005) {
            mtr.setPower(target);
            lastTransferPower = target;
        }
    }

    public void stop_transfer() {
        if (lastTransferPower != 0) {
            mtr.setPower(0);
            lastTransferPower = 0;
        }
    }

    public void farassus(){
        servo_faras.setPosition(0.81);
    }
    public void farasjos(){
        servo_faras.setPosition(0.29);
    }
}
