package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Module.OuttakeModule.act_outtake;
import static org.firstinspires.ftc.teamcode.Module.OuttakeModule.rampa_reper;
import static org.firstinspires.ftc.teamcode.Module.OuttakeModule.target_velocity;
import static org.firstinspires.ftc.teamcode.Module.PinpointModule.distanta;
import static org.firstinspires.ftc.teamcode.Module.TurretModule.decalation;
import static org.firstinspires.ftc.teamcode.Module.OuttakeModule.velocity;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Module.IntakeModule;
import org.firstinspires.ftc.teamcode.Module.OuttakeModule;
import org.firstinspires.ftc.teamcode.Module.PinpointModule;
import org.firstinspires.ftc.teamcode.Module.TurretModule;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
@Configurable
public class TeleOp extends OpMode {
    IntakeModule intake;
    OuttakeModule outtake;
    PinpointModule pinpoint;
    TurretModule turret;
    private Follower follower;
    double delta_velocity;
    ElapsedTime timer;
    boolean target_atins = false, lastA = false, rampa_jos = false;
    double viteza_ref;
    public static double PRAG_COBORARE = 45, RAMPA_SUS = 1, RAMPA_JOS = 1.05;

    enum STATE {
        trage,
        numaitrage
    }


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

        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        act_outtake = false;


    }


    @Override
    public void loop() {
        double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

        follower.manual(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        );

        follower.update();
        pinpoint.update_blue();
        outtake.update(voltage);
        turret.update();

        delta_velocity = velocity - target_velocity - 1;

        if (gamepad1.a && !lastA) {
            act_outtake = !act_outtake;
            target_atins = false;
            rampa_jos = false;
            if (!act_outtake) {
                outtake.blocat();
            }
            timer.reset();
        }
        lastA = gamepad1.a;

        if (gamepad1.b) {
            act_outtake = false;
            target_atins = false;
            rampa_jos = false;
            outtake.blocat();
        }

        boolean tragemsinoi = (delta_velocity < 40 && delta_velocity > -11 && act_outtake);

        if (tragemsinoi && !target_atins) {
            outtake.deblocat();
            target_atins = true;
            timer.reset();
        }

        boolean tragere_inceputa = target_atins && timer.milliseconds() > 250;
        if (!tragere_inceputa) {
            viteza_ref = velocity;
        }
        else if (viteza_ref - velocity > PRAG_COBORARE) {
            rampa_jos = true;
        }
        rampa_reper = rampa_jos ? RAMPA_JOS : RAMPA_SUS;

        if (gamepad1.left_bumper) {
            decalation += 0.1 / 320.0;
        }
        if (gamepad1.right_bumper) {
            decalation -= 0.1 / 320.0;
        }

        if (gamepad1.square) {
            pinpoint.reset_blue();
        }

        if (target_atins && timer.milliseconds() > 250) {
            intake.trage_intake(1);
            intake.trage_transfer(1);
        }
        else if (gamepad1.right_trigger > 0.1) {
            intake.trage_intake(1);
            intake.trage_transfer(1);
        }
        else if (gamepad1.left_trigger > 0.1) {
            intake.scuipa_intake(1);
            intake.scuipa_transfer(1);
        }
        else {
            intake.stop_intake();
            intake.stop_transfer();
        }

        telemetry.addData("Rampa jos", rampa_jos);
        telemetry.addData("Drop viteza", viteza_ref - velocity);
        telemetry.addData("Delta velocity", delta_velocity);
        telemetry.addData("Act Outtake", act_outtake);
        telemetry.addData("Distanta", distanta);
        telemetry.addData("Velocity", velocity);
        telemetry.addData("Target Vel", target_velocity);
        telemetry.update();
    }
}
