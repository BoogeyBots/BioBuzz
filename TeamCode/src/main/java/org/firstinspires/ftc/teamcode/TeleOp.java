package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Module.IntakeModule.opritAutomatStall;
import static org.firstinspires.ftc.teamcode.Module.OuttakeModule.act_outtake;
import static org.firstinspires.ftc.teamcode.Module.OuttakeModule.rampa;
import static org.firstinspires.ftc.teamcode.Module.OuttakeModule.rampa_jos;
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
    private List<LynxModule> allHubs;
    double delta_velocity;
    ElapsedTime timer;
    boolean target_atins = false, lastA = false;
    double viteza_ref;
    public static double PRAG_COBORARE = 45, RAMPA_SUS = 1, RAMPA_JOS = 1.05;
    public static double PUTERE_TRANSFER_TRAGERE = 0.80;
    public static double TIMP_DUPA_DEBLOCARE_MS = 200;

    enum STATE {
        trage,
        numaitrage
    }


    @Override
    public void init() {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

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
        rampa_jos = false;
    }


    @Override
    public void loop() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

        follower.manual(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        );

        follower.update();
        pinpoint.update_blue();

        // 1. Citim butonul A INAINTE de outtake.update(voltage), ca target_velocity sa se schimbe imediat
        if (gamepad1.a && !lastA) {
            act_outtake = !act_outtake;
            target_atins = false;
            rampa_jos = false;
            outtake.blocat(); // Tinem blocajul inchis pana atinge target_velocity real
            timer.reset();
        }
        lastA = gamepad1.a;

//        if (gamepad1.b) {
//            act_outtake = false;
//            target_atins = false;
//            rampa_jos = false;
//            outtake.blocat();
//        }

        // 2. Actualizam outtake-ul (acum stie deja daca act_outtake e true si seteaza target_velocity din tabel)
        outtake.update(voltage);
        turret.update();

        delta_velocity = velocity - target_velocity - 1;

        // 3. Asteptam minim 80ms de la apasarea lui A si ca viteza sa ajunga la noul target_velocity
        boolean tragemsinoi = (act_outtake && timer.milliseconds() > 80 && delta_velocity < 35 && delta_velocity > -12);

        if (tragemsinoi && !target_atins) {
            outtake.deblocat();
            target_atins = true;
            rampa_jos = false;
            timer.reset();
        }

        // 4. Dupa ce s-a deschis servo_blocaj si a pornit transferul, activam regresia rampei cand scade viteza
        boolean tragere_inceputa = target_atins && timer.milliseconds() > TIMP_DUPA_DEBLOCARE_MS;
        if (!tragere_inceputa) {
            viteza_ref = target_velocity;
        }
        else if (target_velocity - velocity > PRAG_COBORARE) {
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

        if (tragere_inceputa) {
            intake.trage_intake(1);
            intake.trage_transfer_tragere(PUTERE_TRANSFER_TRAGERE);
        }
        else if (gamepad1.right_trigger > 0.1 && !act_outtake) {
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

        telemetry.addData("Transfer Stall (Oprit)", opritAutomatStall);
        telemetry.addData("Rampa curenta", rampa);
        telemetry.addData("Rampa jos (Regresie activa)", rampa_jos);
        telemetry.addData("Drop fata de Target", target_velocity - velocity);
        telemetry.addData("Delta velocity", delta_velocity);
        telemetry.addData("Act Outtake", act_outtake);
        telemetry.addData("Target Atins", target_atins);
        telemetry.addData("Distanta", distanta);
        telemetry.addData("Velocity", velocity);
        telemetry.addData("Target Vel", target_velocity);
        telemetry.update();
    }
}
