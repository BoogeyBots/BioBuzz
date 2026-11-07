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

    // Control servo_rampa (pozitie initiala 0.68 -> pozitie tinta 1.0)
    public static double posRampaInitial = OuttakeModule.POS_RAMPA_INITIAL;
    public static double posRampaTarget = OuttakeModule.POS_RAMPA_TARGET;
    public static boolean folosescUndershoot = true;
    public static double marjaUndershoot = 40.0;
    public static double delayRampaMs = 150.0;
    public static boolean fallbackTimerActiv = true;
    public static double fallbackTimeoutMs = 600.0;

    // Putere transfer la apasarea pe trigger (configurabila in Panels)
    public static double putereTransferTrigger = 0.5;

    private ElapsedTime timerRampa = new ElapsedTime();
    private boolean rampaCoborata = false;

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
    public void start() {
        // La pornirea TeleOp-ului, shooter-ul porneste direct la viteza vel
        OuttakeModule.target_velocity = vel;
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

        // Buton A: Pornire / Oprire secventa de tragere
        if (gamepad1.a && !previousA) {
            act_outtake = !act_outtake;
            deblocatActivat = false;
            feedPornit = false;
            rampaCoborata = false;

            outtake.resetBoost();
            outtake.blocat();
            outtake.setRampa(posRampaInitial);

            if (!act_outtake) {
                intake.stop_intake();
                intake.stop_transfer();
            }
        }
        previousA = gamepad1.a;

        // Verificam daca viteza flywheel-ului a atins target-ul
        double delta_velocity = velocity - target_velocity;
        tragemsinoi = (act_outtake && target_velocity > 0 && Math.abs(delta_velocity) < 40);

        if (act_outtake) {
            // Se deblocheaza servo blocaj doar cand ajunge la target velocity
            if (tragemsinoi && !deblocatActivat) {
                outtake.deblocat();
                timerDeblocat.reset();
                deblocatActivat = true;
            }

            // Dupa ce se da blocajul (delay pentru miscare fizica a servo-ului), pornesc motoarele
            if (deblocatActivat && timerDeblocat.milliseconds() >= 100) {
                if (!feedPornit) {
                    feedPornit = true;
                    timerRampa.reset();
                }
            }
        } else {
            // Cand nu se trage, blocajul ramane pus
            outtake.blocat();
            deblocatActivat = false;
            feedPornit = false;
        }

        // Logica pozitionare rampa la tragere
        if (act_outtake) {
            if (feedPornit && !rampaCoborata) {
                boolean declanseazaRampa = false;

                if (folosescUndershoot) {
                    boolean undershoot = velocity < (target_velocity - marjaUndershoot);
                    boolean timeoutSiguranta = fallbackTimerActiv && (timerRampa.milliseconds() >= fallbackTimeoutMs);
                    declanseazaRampa = undershoot || timeoutSiguranta;
                } else {
                    declanseazaRampa = timerRampa.milliseconds() >= delayRampaMs;
                }

                if (declanseazaRampa) {
                    outtake.setRampa(posRampaTarget);
                    rampaCoborata = true;
                } else {
                    outtake.setRampa(posRampaInitial);
                }
            } else if (rampaCoborata) {
                outtake.setRampa(posRampaTarget);
            } else {
                outtake.setRampa(posRampaInitial);
            }
        } else {
            outtake.setRampa(posRampaInitial);
        }

        // Control motoare Intake si Transfer
        if (act_outtake) {
            if (feedPornit) {
                // Automatizarea cand motoarele sunt la target velocity si blocajul e dat:
                intake.trage_intake(1.0);
                intake.trage_transfer(1.0);
            } else {
                // Pana atinge target velocity, motoarele stau oprite
                intake.stop_intake();
                intake.stop_transfer();
            }
        } else if (gamepad1.right_trigger > 0.05) {
            // Right trigger (cum era inainte: proportional cu trigger-ul la intake)
            intake.trage_intake(gamepad1.right_trigger);
            intake.trage_transfer(putereTransferTrigger);
        } else if (gamepad1.left_trigger > 0.05) {
            // Left trigger (evacuare / scuipa)
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

        if (gamepad1.dpad_left) {
            outtake.blocat();
        } else if (gamepad1.dpad_right) {
            outtake.deblocat();
        }

        telemetry.addData("Act Outtake", act_outtake);
        telemetry.addData("Poz Rampa Initial", posRampaInitial);
        telemetry.addData("Poz Rampa Target", posRampaTarget);
        telemetry.addData("Rampa Pozitie Curenta", rampaCoborata ? posRampaTarget : posRampaInitial);
        telemetry.addData("Rampa Coborata", rampaCoborata);
        telemetry.addData("Feed Pornit", feedPornit);
        telemetry.addData("Distanta", distanta);
        telemetry.addData("Velocity", velocity);
        telemetry.addData("Target Vel", target_velocity);
        telemetry.addData("Boost Vel", OuttakeModule.current_boost);
        telemetry.addData("Delta Vel", delta_velocity);
        telemetry.addData("Transfer Voltage", IntakeModule.transferVoltage);
        telemetry.addData("Cx", currentX);
        telemetry.addData("Cy", currentY);
        telemetry.update();
    }
}
