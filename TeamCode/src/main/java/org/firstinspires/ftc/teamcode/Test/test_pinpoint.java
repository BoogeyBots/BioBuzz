package org.firstinspires.ftc.teamcode.Test;

import static org.firstinspires.ftc.teamcode.Module.PinpointModule.currentHeading;
import static org.firstinspires.ftc.teamcode.Module.PinpointModule.currentX;
import static org.firstinspires.ftc.teamcode.Module.PinpointModule.currentY;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Module.PinpointModule;
import org.firstinspires.ftc.teamcode.Module.TurretModule;
import org.firstinspires.ftc.teamcode.pedro.Constants;
@TeleOp
public class test_pinpoint extends OpMode {
    PinpointModule p;
    TurretModule t;
    private Follower follower;

    @Override
    public void init() {
        p = new PinpointModule(hardwareMap);
        t = new TurretModule(hardwareMap);

        t.init();
        p.init();

        follower = Constants.create(hardwareMap);
    }

    @Override
    public void loop() {
        follower.manual(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        );

        follower.update();
        p.update();
        t.update();

        telemetry.addData("cx", currentX);
        telemetry.addData("cy", currentY);
        telemetry.addData("ch", currentHeading);
        telemetry.update();
    }
}
