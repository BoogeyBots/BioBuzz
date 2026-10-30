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

    // Control servo_rampa (pozitie initiala 1.0 -> miscare spre pozitia tinta ex: 0.8 la scaderea vitezei)
    public static double posRampaInitial = 1.0;      // Pozitia initiala (de repaus) a rampei (default 1.0)
    public static double posRampaTarget = 0.8;       // Pozitia tinta a rampei (reglabila in Panels, ex: 0.8)
    public static boolean folosescUndershoot = true; // true = foloseste scaderea de viteza (undershoot/drop), false = doar timer
    public static double marjaUndershoot = 40.0;     // Cate unitati de viteza sub target declanseaza miscarea rampei
    public static double delayRampaMs = 150.0;       // Timer delay in ms de la alimentarea bilei daca folosescUndershoot == false
    public static boolean fallbackTimerActiv = true; // Backup de siguranta pe timer daca undershoot-ul nu e detectat la timp
    public static double fallbackTimeoutMs = 600.0;  // Timeout fallback in milisecunde pentru backup de siguranta

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
            deblocatActivat = false;
            feedPornit = false;
            rampaCoborata = false;

            if (act_outtake) {
                outtake.resetBoost();
                // S-a pornit shooter-ul -> blocam servo_blocaj si punem rampa la pozitia initiala (1.0)
                outtake.blocat();
                outtake.setRampa(posRampaInitial);
                intake.trage_intake(0.6);
                intake.trage_transfer(1.0);
            } else {
                outtake.resetBoost();
                // S-a oprit shooter-ul -> blocam si rampa revine la pozitia initiala (1.0)
                outtake.blocat();
                outtake.setRampa(posRampaInitial);
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
            if (!feedPornit) {
                feedPornit = true;
                timerRampa.reset();
            }
        }

        // Logica pozitionare rampa
        if (act_outtake) {
            if (feedPornit && !rampaCoborata) {
                boolean declanseazaRampa = false;

                if (folosescUndershoot) {
                    // Viteza a scazut sub target cu marja configurata (cand bila atinge flywheel-ul)
                    boolean undershoot = velocity < (target_velocity - marjaUndershoot);
                    // Backup pe timer in milisecunde: daca undershoot-ul nu e atins in timp util
                    boolean timeoutSiguranta = fallbackTimerActiv && (timerRampa.milliseconds() >= fallbackTimeoutMs);
                    declanseazaRampa = undershoot || timeoutSiguranta;
                } else {
                    // Mod doar pe timer (in milisecunde de la start feed)
                    declanseazaRampa = timerRampa.milliseconds() >= delayRampaMs;
                }

                if (declanseazaRampa) {
                    outtake.setRampa(posRampaTarget); // Rampa se duce la pozitia tinta (default 0.8 sau aleasa in Panels)
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
            // Cand shooter-ul este oprit, rampa ramane la pozitia initiala (1.0)
            outtake.setRampa(posRampaInitial);
        }

        if (act_outtake) {
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
        telemetry.addData("Poz Rampa Initial", posRampaInitial);
        telemetry.addData("Poz Rampa Target", posRampaTarget);
        telemetry.addData("Rampa Pozitie Curenta", rampaCoborata ? posRampaTarget : posRampaInitial);
        telemetry.addData("Rampa Coborata", rampaCoborata);
        telemetry.addData("Timer Rampa (ms)", timerRampa.milliseconds());
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
