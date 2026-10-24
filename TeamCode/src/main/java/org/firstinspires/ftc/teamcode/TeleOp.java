package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Module.OuttakeModule.act_outtake;
import static org.firstinspires.ftc.teamcode.Module.OuttakeModule.target_velocity;
import static org.firstinspires.ftc.teamcode.Module.OuttakeModule.velocity;
import static org.firstinspires.ftc.teamcode.Module.PinpointModule.currentX;
import static org.firstinspires.ftc.teamcode.Module.PinpointModule.currentY;
import static org.firstinspires.ftc.teamcode.Module.PinpointModule.distanta;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Module.IntakeModule;
import org.firstinspires.ftc.teamcode.Module.OuttakeModule;
import org.firstinspires.ftc.teamcode.Module.PinpointModule;
import org.firstinspires.ftc.teamcode.Module.TurretModule;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
@Configurable
public class TeleOp extends OpMode {
    IntakeModule intake;
    OuttakeModule outtake;
    PinpointModule pinpoint;
    TurretModule turret;
    private Follower follower;
    public static double vel = 700;
    private boolean previousA = false;
    boolean tragemsinoi = false;

    private ElapsedTime timerDeblocat = new ElapsedTime();
    private boolean deblocatActivat = false;
    private boolean feedPornit = false;

    @Override
    public void init() {
        intake = new IntakeModule(hardwareMap);
        outtake = new OuttakeModule(hardwareMap);
        pinpoint = new PinpointModule(hardwareMap);
        turret = new TurretModule(hardwareMap);

        intake.init();
        outtake.init();
        pinpoint.init();
        turret.init();

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
        pinpoint.update();
        outtake.update(vel);
        turret.update();

        if (gamepad1.a && !previousA) {
            act_outtake = !act_outtake;
            outtake.blocat();
            deblocatActivat = false;
            feedPornit = false;

            if (!act_outtake) {
                intake.stop_intake();
                intake.stop_transfer();
            }
        }
        previousA = gamepad1.a;

        // Verificam viteza shooter-ului
        double delta_velocity = velocity - target_velocity - 1;
        tragemsinoi = (delta_velocity < 40 && delta_velocity > -11 && act_outtake);

        if (tragemsinoi && !deblocatActivat) {
            outtake.deblocat();
            timerDeblocat.reset();
            deblocatActivat = true;
        }

        if (deblocatActivat && timerDeblocat.milliseconds() >= 100 && act_outtake) {
            feedPornit = true;
        }

        if (feedPornit && act_outtake) {
            intake.trage_intake(1.0);
            intake.trage_transfer(1.0);
        } else if (gamepad1.right_trigger > 0.05) {
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

        telemetry.addData("Act Outtake", act_outtake);
        telemetry.addData("Feed Pornit", feedPornit);
        telemetry.addData("Distanta", distanta);
        telemetry.addData("Velocity", velocity);
        telemetry.addData("Cx", currentX);
        telemetry.addData("Cy", currentY);
        telemetry.update();
    }
}
