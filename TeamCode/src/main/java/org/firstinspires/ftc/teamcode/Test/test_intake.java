package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Module.IntakeModule;

@TeleOp(name = "test intake")
public class test_intake extends OpMode {

    private IntakeModule intake;

    @Override
    public void init() {
        intake = new IntakeModule(hardwareMap);
        intake.init();
    }

    @Override
    public void loop() {
        if (gamepad1.right_trigger > 0.05) {
            intake.trage_intake(gamepad1.right_trigger);
            intake.trage_transfer(gamepad1.right_trigger);
        } else if (gamepad1.left_trigger > 0.05) {
            intake.scuipa_intake(gamepad1.left_trigger);
            intake.scuipa_transfer(gamepad1.left_trigger);
        } else {
            intake.stop_intake();
            intake.stop_transfer();
        }

        if (gamepad1.dpad_up) {
            intake.farassus();
        } else if (gamepad1.dpad_down) {
            intake.farasjos();
        }
    }
}
