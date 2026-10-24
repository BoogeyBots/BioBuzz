package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import static java.lang.Math.abs;


import org.firstinspires.ftc.teamcode.Module.IntakeModule;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

@Disabled
public class IncercareTeleOp extends LinearOpMode {

    IntakeModule intake = null;


    private Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.create(hardwareMap);
        intake = new IntakeModule(hardwareMap);




        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }


        waitForStart();

        while (opModeIsActive()) {
            //loop.reset();
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }


            follower.manual(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            );

            follower.update();
        }
    }
}
