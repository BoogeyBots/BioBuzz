package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@TeleOp
public class test_intake_biobubz extends OpMode {

    int rotatii = 0;
    DcMotorEx mi, mds;

    @Override
    public void init() {
        mi = hardwareMap.get(DcMotorEx.class, "motorintake");
        mds = hardwareMap.get(DcMotorEx.class, "motortransfer");
    }

    @Override
    public void loop() {
        if(gamepad1.right_trigger > 0.01) {
            mds.setPower(1);
            mi.setPower(-1);
        }

        else if(gamepad1.left_trigger > 0.01) {
            mds.setPower(-1);
            mi.setPower(1);
        }

        else {
            mds.setPower(0);
            mi.setPower(0);
        }

        telemetry.addData("Power:", mds.getPower());
        telemetry.addData("power", mi.getPower());
        telemetry.update();
    }
}
