package org.firstinspires.ftc.teamcode.Test;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@TeleOp
@Configurable
public class Test_motor extends LinearOpMode {
    DcMotorEx motor, motor2;
    public static double pow1, pow2;
    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "motor_intake");
        motor2 = hardwareMap.get(DcMotorEx.class, "motor_transfer");

        waitForStart();

        while (opModeIsActive()) {
            motor.setPower(pow1);
            motor2.setPower(pow2);
        }

    }
}
