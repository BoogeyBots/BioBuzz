package org.firstinspires.ftc.teamcode.Test;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Configurable
@TeleOp
public class test_servo extends LinearOpMode {
    Servo servo;
    public static double poz;
    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "servo_faras");

        waitForStart();

        while (opModeIsActive()) {
            servo.setPosition(poz);
        }

    }
}
